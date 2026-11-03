#if defined(ENABLE_DEBUG) && !defined(ENABLE_DEBUG_RAPI)
#undef ENABLE_DEBUG
#endif

#include <Arduino.h>
#include "RapiSender.h"
#include "MicroDebug.h"
#include <ctype.h>
#include <string.h>

#define dbgprint(s) DBUG(s)
#define dbgprintln(s) DBUGLN(s)
#ifdef ENABLE_DEBUG
#define DBG
#endif

static CommandItem commandQueueItems[RAPI_MAX_COMMANDS];

// convert 2-digit hex string to uint8_t
uint8_t
htou8(const char *s) {
  uint8_t u = 0;
  for (int i = 0; i < 2; i++) {
    char c = s[i];
    if (c != '\0') {
      if (i == 1)
        u <<= 4;
      if ((c >= '0') && (c <= '9')) {
        u += c - '0';
      } else if ((c >= 'A') && (c <= 'F')) {
        u += c - 'A' + 10;
      }
      //      else if ((c >= 'a') && (c <= 'f')) {
      //        u += c - 'a' + 10;
      //      }
      else {
        // invalid character received
        return 0;
      }
    }
  }
  return u;
}

RapiSender::RapiSender(Stream * stream) :
  _stream(stream),
  _sent(0),
  _success(0),
  _connected(false),
  _sequenceId(RAPI_INVALID_SEQUENCE_ID),
  _flags(0),
  _tokenCnt(0),
  _tokens{},
  _onRapiEvent(nullptr),
  _commandQueue(commandQueueItems, RAPI_MAX_COMMANDS),
  _completeHandler(nullptr),
  _timeout(0),
  _waitingForReply(false),
  _holdUntil(0),
  _holding(false),
  _holdMaxQueued(RAPI_HOLD_MAX_QUEUED),
  _priority(false),
  _inFlight(),
  _respBuf{},
  _respBufOrig{}
{
}

void RapiSender::holdSend(unsigned long ms, size_t maxQueued)
{
  _holdUntil = millis() + ms;
  _holding = true;
  _holdMaxQueued = maxQueued > queueCapacity() ? queueCapacity() : maxQueued;

  // Drop priority commands left over from before this hold (e.g. boot reads
  // for a controller that has just restarted again); the caller re-issues
  // them. Ordinary commands keep their place. Handlers run after the queue is
  // rebuilt, so one that queues a new command cannot disturb the scan.
  _inFlight.retries = 0;
  RapiCommandCompleteHandler cancelled[RAPI_MAX_COMMANDS];
  size_t nCancelled = 0;
  size_t n = _commandQueue.used();
  for(size_t i = 0; i < n; i++) {
    CommandItem cmd;
    if(!_commandQueue.pop(cmd)) {
      break;
    }
    if(cmd.priority) {
      cancelled[nCancelled++] = cmd.handler;
    } else {
      _commandQueue.push(cmd);
    }
  }
  for(size_t i = 0; i < nCancelled; i++) {
    if(nullptr != cancelled[i]) {
      cancelled[i](RAPI_RESPONSE_CANCELLED);
    }
  }
}

bool RapiSender::_retryInFlight(int result)
{
  if(!_inFlight.priority ||
     !(RAPI_RESPONSE_TIMEOUT == result || RAPI_RESPONSE_BAD_CHECKSUM == result ||
       RAPI_RESPONSE_INVALID_RESPONSE == result)) {
    return false;
  }
  if(0 == _inFlight.retries) {
    DBUGF("RAPI: giving up on %s (%d)", _inFlight.command, result);
    return false;
  }
  _inFlight.retries--;
  DBUGF("RAPI: retrying %s (%d)", _inFlight.command, result);
  if(_sendHeld()) {
    // Requeue (behind what is already waiting) rather than send into a hold.
    CommandItem retry = _inFlight;
    retry.handler = _completeHandler;
    if(!_commandQueue.push(retry)) {
      return false;
    }
    _inFlight = CommandItem();
    _completeHandler = nullptr;
    _waitingForReply = false;
    return true;
  }
  // Re-send now, keeping its place ahead of the queue. It gets a fresh
  // sequence id and a fresh timeout, so a late reply to the earlier attempt
  // is dropped as stale.
  _sendCmd(_inFlight.command);
  _timeout = millis() + _inFlight.timeout;
  return true;
}

bool RapiSender::_sendHeld()
{
  // Signed difference so the hold survives the millis() rollover.
  if(_holding && (long)(millis() - _holdUntil) < 0) {
    return true;
  }
  _holding = false;
  return false;
}

void RapiSender::_sendNextCmd()
{
  if(_sendHeld()) {
    // Commands stay queued; loop() sends them once the hold expires. Their
    // timeouts start when they are actually sent.
    return;
  }
  CommandItem cmd;
  if(_commandQueue.pop(cmd))
  {
    _sendCmd(cmd.command);
    _completeHandler = cmd.handler;
    _inFlight = cmd;
    _inFlight.handler = nullptr; // _completeHandler owns it
    _timeout = millis() + cmd.timeout;
    _waitingForReply = true;
  }
}

// return = 0 = OK
//        = 1 = command will cause buffer overflow
void
RapiSender::_sendCmd(const char *cmdstr) {
  // a caller's own "^HH" checksum (e.g. a /r client) is dropped;
  // our checksum below covers the whole line, including any sequence id.
  const char *caret = strchr(cmdstr, '^');
  size_t len = caret ? (size_t)(caret - cmdstr) : strlen(cmdstr);
  _stream->write((const uint8_t *)cmdstr, len);
  dbgprint(cmdstr);

  uint8_t chk = 0;
  for (size_t i = 0; i < len; i++) {
    chk ^= cmdstr[i];
  }

  // remember whether this is the version query (see _tokenize)
  if (len >= 3 && !strncmp(cmdstr, "$GV", 3) && (len == 3 || cmdstr[3] == ' '))
    _flags |= RSF_CMD_IS_VERSION;
  else
    _flags &= ~RSF_CMD_IS_VERSION;

  // if the caller already put its own sequence id on the end
  // (" :HH", e.g. a /r pass-through client), use it as-is instead of adding a
  // second one, and expect that id in the reply.
  bool addSeq = true;
  if (_sequenceIdEnabled() && len >= 4 &&
      cmdstr[len-4] == ' ' && cmdstr[len-3] == ESRAPI_SOS &&
      isxdigit((unsigned char)cmdstr[len-2]) && isxdigit((unsigned char)cmdstr[len-1])) {
    char hex[3] = { (char)toupper((unsigned char)cmdstr[len-2]), (char)toupper((unsigned char)cmdstr[len-1]), 0 };
    _sequenceId = htou8(hex);
    addSeq = false;
  }

  _sendTail(chk, addSeq);

  _sent++;
}

void RapiSender::_sendTail(uint8_t chk, bool addSeq) {
  if (_sequenceIdEnabled() && addSeq) {
    if (++_sequenceId == RAPI_INVALID_SEQUENCE_ID)
      ++_sequenceId;
    sprintf(_respBuf, " %c%02X", ESRAPI_SOS, (unsigned) _sequenceId);
    const char *s = _respBuf;
    while (*s) {
      chk ^= *(s++);
    }
    _stream->print(_respBuf);
    dbgprint(_respBuf);
  }

  sprintf(_respBuf, "^%02X%c", (unsigned) chk, ESRAPI_EOC);
  _stream->print(_respBuf);
  dbgprintln(_respBuf);
  _stream->flush();

  *_respBuf = 0;
}

// return = 0 = OK
//        = 1 = bad checksum
//        = 2 = bad sequence id
int
RapiSender::_tokenize() {
  uint8_t chk = 0;
  char *s = _respBuf;
  dbgprint("resp: ");
  dbgprintln(_respBuf);

  while (*s != '^' && *s != '\0') {
    chk ^= *(s++);
  }
  if (*s == '^') {
    uint8_t rchk = htou8(s + 1);
    if (rchk != chk) {
      _tokenCnt = 0;
#ifdef DBG
      sprintf(_respBuf, "bad chk %x %x %s", rchk, chk, s);
      dbgprintln(_respBuf);
#endif
      return RAPI_RESPONSE_BAD_CHECKSUM;
    }
    *s = '\0';
  }

  _tokenCnt = 0;
  s = _respBuf;
  while (*s) {
    _tokens[_tokenCnt++] = s++;
    if (_tokenCnt == RAPI_MAX_TOKENS)
      break;

    while (*s && (*s != ' ')) {
      s++;
    }
    if (*s == ' ') {
      *(s++) = '\0';
    }
  }

  // the controller sends the sequence id as its own last token
  // (" :HH", open_evse rapi_proc.cpp appendSequenceId). Strip it, and for
  // command replies ($OK/$NK) reject it if it isn't the id we just sent.
  // Async events ($ST/$AT/...) and controller-initiated commands ($WF...) are
  // not replies to us, so their id (if any) is just dropped.
  bool isReply = _tokenCnt > 0 &&
    (!strcmp(_tokens[0], "$OK") || !strcmp(_tokens[0], "$NK"));
  bool hasSeq = _tokenCnt > 1 && _tokens[_tokenCnt-1][0] == ESRAPI_SOS;
  if (_sequenceIdEnabled() && isReply && !hasSeq) {
    int major, minor, patch;
    // "$OK <fw version> <RAPI version>" with no id, before the controller
    // has ever echoed one: the controller does not support ids (older
    // firmware, or an implementation that leaves them out whatever RAPI
    // version it reports). Only a well-formed version reply counts.
    if (!(_flags & RSF_SEQUENCE_ID_SEEN) && (_flags & RSF_CMD_IS_VERSION) &&
        !strcmp(_tokens[0], "$OK") && _tokenCnt >= 3 &&
        parseVersion(_tokens[2], major, minor, patch)) {
      // Turn id checking off for this session and accept the reply.
      enableSequenceId(0);
    } else {
      // Any other id-less reply (a non-version reply, a malformed version
      // reply, or any reply once the controller has echoed an id) cannot be
      // matched to the command we sent.
      // Drop it and keep ids on; the caller's timeout/retry re-asks.
#ifdef DBG
      dbgprintln("missing seqid");
#endif
      _tokenCnt = 0;
      return RAPI_RESPONSE_BAD_SEQUENCE_ID;
    }
  }
  if (hasSeq) {
    uint8_t seqid = htou8(_tokens[_tokenCnt-1] + 1);
    _tokenCnt--;
    if (_sequenceIdEnabled() && isReply && seqid != _sequenceId) {
#ifdef DBG
      dbgprint("bad seqid ");
      dbgprintln(seqid);
#endif
      _tokenCnt = 0;
      return RAPI_RESPONSE_BAD_SEQUENCE_ID;
    }
    if (isReply) {
      _flags |= RSF_SEQUENCE_ID_SEEN;
    }
  }

  return RAPI_RESPONSE_OK;
}

void RapiSender::_commandComplete(int result)
{
  // Hazard, not currently reachable: _waitingForReply stays set for the whole
  // of the handler call, so a handler that itself calls sendCmdSync() or
  // flush() re-enters loop() and arrives back here, invoking the same handler
  // a second time. _sendNextCmd() would then assign to _completeHandler while
  // its operator() is still on the stack. No caller does this today; fixing it
  // properly means deciding whether a nested synchronous send should be
  // allowed at all, which is a bigger change than this branch should make.
  if(_waitingForReply && _retryInFlight(result)) {
    if(!_waitingForReply) {
      _sendNextCmd(); // requeued: nothing in flight now
    }
    return;
  }
  if(_waitingForReply) {
    _inFlight = CommandItem();
    if(nullptr != _completeHandler) {
      _completeHandler(result);
    }
    _waitingForReply = false;
  }
  _sendNextCmd();
}

void
RapiSender::sendCmd(const char *cmdstr, RapiCommandCompleteHandler callback, unsigned long timeout) {
  CommandItem cmd;

  // Reject rather than truncate. A truncated RAPI command is still a valid
  // command with a different meaning -- "$SC 32" cut to "$SC 3" would set a
  // different current -- so silently shortening one is worse than refusing it.
  if(strlen(cmdstr) >= sizeof(cmd.command)) {
    if(nullptr != callback) {
      callback(RAPI_RESPONSE_CMD_TOO_LONG);
    }
    return;
  }

  // Id 00 is reserved as "no id": the controller would run the command but
  // reply without an id. Refuse a caller-supplied " :00" before sending it.
  if(_sequenceIdEnabled()) {
    const char *caret = strchr(cmdstr, '^');
    size_t len = caret ? (size_t)(caret - cmdstr) : strlen(cmdstr);
    if(len >= 4 && !strncmp(cmdstr + len - 4, " :00", 4)) {
      if(nullptr != callback) {
        callback(RAPI_RESPONSE_BAD_SEQUENCE_ID);
      }
      return;
    }
  }

  strcpy(cmd.command, cmdstr);
  cmd.handler = callback;
  cmd.timeout = timeout;
  cmd.priority = _priority;
  cmd.retries = _priority ? RAPI_PRIORITY_RETRIES : 0;

  // During a hold keep most of the queue free for priority commands.
  bool capped = !_priority && _sendHeld() &&
                _commandQueue.used() >= _holdMaxQueued;
  if(!capped && _commandQueue.push(cmd)) {
    if(!_waitingForReply) {
      _sendNextCmd();
    }
  } else if(nullptr != callback) {
    callback(RAPI_RESPONSE_QUEUE_FULL);
  }
}

void
RapiSender::sendCmd(String &cmdstr, RapiCommandCompleteHandler callback, unsigned long timeout) {
  return sendCmd(cmdstr.c_str(), callback, timeout);
}

void
RapiSender::sendCmd(const __FlashStringHelper *cmdstr, RapiCommandCompleteHandler callback, unsigned long timeout) {
  // One byte longer than the queue's buffer, so that a string too long to
  // queue still ends up too long here and is rejected by the char* overload
  // rather than arriving silently truncated to a valid-looking command.
  char cmd[RAPI_CMD_BUFLEN + 1];
  strncpy_P(cmd, (PGM_P)cmdstr, sizeof(cmd));
  cmd[sizeof(cmd) - 1] = '\0';
  return sendCmd(cmd, callback, timeout);
}

int
RapiSender::sendCmdSync(const char *cmdstr, unsigned long timeout) {
  struct SendCmdSyncData
  {
    int ret;
    bool finished;
  } resultData = {0, false};
  SendCmdSyncData *result = &resultData;

  // The caller's timeout was previously dropped here, so every synchronous
  // command used the RAPI_TIMEOUT_MS default no matter what was asked for.
  sendCmd(cmdstr, [result](int ret) {
    result->ret = ret;
    result->finished = true;
  }, timeout);

  while(!result->finished) {
    loop();
  }

  return result->ret;
}

int
RapiSender::sendCmdSync(String &cmdstr, unsigned long timeout)
{
  return sendCmdSync(cmdstr.c_str(), timeout);
}

int
RapiSender::sendCmdSync(const __FlashStringHelper *cmdstr, unsigned long timeout) {
  char cmd[RAPI_CMD_BUFLEN + 1];
  strncpy_P(cmd, (PGM_P)cmdstr, sizeof(cmd));
  cmd[sizeof(cmd) - 1] = '\0';
  return sendCmdSync(cmd, timeout);
}

/*
 * return values:
 * -1= timeout
 * 0= success
 * 1=$NK
 * 2=invalid RAPI response
 * 3=cmdstr too long
 * 4=bad checksum
 * 5=bad sequence ID
*/
int
RapiSender::_waitForResult(unsigned long timeout) {
  unsigned long mss = millis();

  _tokenCnt = 0;
  *_respBuf = 0;
  int bufpos = 0;
  do {
    int bytesavail = _stream->available();
    if (bytesavail) {
      for (int i = 0; i < bytesavail; i++) {
        char c = _stream->read();
        if (!bufpos && (c != ESRAPI_SOC)) {
          // wait for start character
          continue;
        } else if (bufpos && (c == ESRAPI_EOC)) {
          _respBuf[bufpos] = '\0';
          // Save the original response
          strncpy(_respBufOrig, _respBuf, RAPI_BUFLEN);
          int ret = _tokenize();
          if (RAPI_RESPONSE_OK == ret)
            break;

          return ret;
        } else {
          _respBuf[bufpos++] = c;
          if (bufpos >= (RAPI_BUFLEN - 1))
            return RAPI_RESPONSE_BUFFER_OVERFLOW;
        }
      }
    }
  } while (!_tokenCnt && ((millis() - mss) < timeout));

#ifdef DBG
  dbgprint("TOKENCNT: ");
  dbgprintln(_tokenCnt);
  for (int i = 0; i < _tokenCnt; i++) {
    dbgprintln(_tokens[i]);
  }
  dbgprintln("");
#endif

  if (_tokenCnt > 0) {
    if (!strcmp(_tokens[0], "$OK")) {
      _success++;
      _connected = true;
      return RAPI_RESPONSE_OK;
    } else if (!strcmp(_tokens[0], "$NK")) {
      return RAPI_RESPONSE_NK;
    } else if (!strcmp(_tokens[0],"$WF") ||
               !strcmp(_tokens[0],"$ST") ||
               !strncmp(_tokens[0],"$A",2))
    {
      return RAPI_RESPONSE_ASYNC_EVENT;
    } else { // not OK or NK
      return RAPI_RESPONSE_INVALID_RESPONSE;
    }
  } else { // !_tokenCnt
    _connected = false;
    return RAPI_RESPONSE_TIMEOUT;
  }
}

void
RapiSender::enableSequenceId(uint8_t tf) {
  if (tf) {
    // Random start, so a reply left over from before a restart is unlikely
    // to carry the id of our first commands.
#if defined(EPOXY_DUINO) || defined(UNIX_HOST_DUINO)
    // host builds (simulator/tests) may define ESP32/ESP8266 without the SDK
    _sequenceId = (uint8_t) (micros() ^ (millis() >> 3));
#elif defined(ESP32)
    _sequenceId = (uint8_t) esp_random();
#elif defined(ESP8266)
    _sequenceId = (uint8_t) RANDOM_REG32;
#else
    _sequenceId = (uint8_t) (micros() ^ (millis() >> 3));
#endif
    if (_sequenceId == RAPI_INVALID_SEQUENCE_ID) _sequenceId = 1;
    // Start a new session: whether this controller echoes ids is not known
    // until it does again (e.g. it was replaced or reflashed since).
    _flags &= ~RSF_SEQUENCE_ID_SEEN;
    _flags |= RSF_SEQUENCE_ID_ENABLED;
  } else {
    _sequenceId = RAPI_INVALID_SEQUENCE_ID;
    _flags &= ~(RSF_SEQUENCE_ID_ENABLED | RSF_SEQUENCE_ID_SEEN);
  }
}

void
RapiSender::loop()
{
  if(_stream->available())
  {
    int ret = _waitForResult(RAPI_READ_TIMEOUT_MS);
    if(RAPI_RESPONSE_BAD_SEQUENCE_ID == ret) {
      // a stale reply to an earlier (timed-out) command. Drop it
      // and keep waiting for the reply to the current one (or its timeout).
      DBUGLN("RAPI: dropped reply with wrong sequence id");
    } else if(RAPI_RESPONSE_ASYNC_EVENT == ret) {
      // async EVSE state transition or WiFi event
      if(nullptr != _onRapiEvent) {
        _onRapiEvent();
      }
    } else {
      _commandComplete(ret);
    }
  }
  else if (_waitingForReply && millis() >= _timeout)
  {
    _commandComplete(RAPI_RESPONSE_TIMEOUT);
  }
  // Release commands queued while sending was held (see holdSend()).
  if(!_waitingForReply && hasPendingCommands()) {
    _sendNextCmd();
  }
}

bool RapiSender::flush(unsigned long timeout)
{
  DBUGLN("RapiSender::flush()");
  unsigned long start = millis();
  while(hasPendingCommands() || _waitingForReply)
  {
    DBUGVAR(hasPendingCommands());
    DBUGVAR(_waitingForReply);
    // Subtract before comparing so the wait survives the millis() rollover.
    if(millis() - start >= timeout) {
      DBUGLN("RapiSender::flush() gave up");
      return false;
    }
    loop();
  }
  return true;
}
