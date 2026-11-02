#pragma once
#include <Stream.h>
#include <functional>
#include <new>
#include <type_traits>
#include <utility>
#include <string.h>

#include "queue.h"

// only enable if RAPI ver
#define RAPI_SEQUENCE_ID

#define RAPI_INVALID_SEQUENCE_ID 0

#define RAPI_TIMEOUT_MS 500
#define RAPI_READ_TIMEOUT_MS 20
#define RAPI_BUFLEN 100
// Longest reply: $OK plus 9 fields ($GL with stuck-relay count), plus the
// " :HH" sequence id token, plus one spare.
#define RAPI_MAX_TOKENS 12

#define ESRAPI_SOC '$' // start of command
#define ESRAPI_EOC 0xd // CR end of command
#define ESRAPI_SOS ':' // start of sequence id

#ifndef RAPI_MAX_COMMANDS
#define RAPI_MAX_COMMANDS 10
#endif

// Ceiling on a single flush(). A full queue of commands that a controller
// never answers takes RAPI_MAX_COMMANDS * RAPI_TIMEOUT_MS to drain -- 5 s at
// the defaults, which is exactly the arduino-esp32 task watchdog period -- and
// completion handlers may queue more work as they run, so the drain has no
// upper bound at all. flush() runs on the caller's task, which on the ESP32
// gateway is the Arduino loop task, sometimes inside a web request handler.
// Bound it well clear of the watchdog and let whatever is left drain from the
// caller's normal loop() instead.
#ifndef RAPI_FLUSH_TIMEOUT_MS
#define RAPI_FLUSH_TIMEOUT_MS 2000
#endif

// Longest queued command, including the terminator. The library's own commands
// are built in stack buffers of at most 64 bytes; the only unbounded source is
// a command relayed from a web/MQTT client, which is rejected with
// RAPI_RESPONSE_CMD_TOO_LONG rather than truncated.
#ifndef RAPI_CMD_BUFLEN
#define RAPI_CMD_BUFLEN 80
#endif

// Inline storage for a queued completion callback. The callbacks in this
// library are `[this, callback]` lambdas, where `callback` is a std::function:
// on a 32-bit target that is a 4-byte `this` plus a 16-byte std::function, so
// 20 bytes, and 40 leaves room without being wasteful.
//
// Both halves scale with the pointer width, so size the buffer that way rather
// than hard-coding the 32-bit answer. A fixed 40 compiles on the ESP32 and
// trips the static_assert below on a 64-bit host, which breaks every native /
// EpoxyDuino build that links this library while the hardware targets stay
// green -- so CI on device builds alone will not catch it.
//
// Anything that still does not fit is a compile error rather than a silent
// heap allocation, which is the whole point of the inline storage.
#ifndef RAPI_HANDLER_CAPACITY
#define RAPI_HANDLER_CAPACITY (sizeof(void *) * 10)
#endif

#define RAPI_RESPONSE_NOT_CONNECTED          -4
#define RAPI_RESPONSE_QUEUE_FULL             -3
#define RAPI_RESPONSE_BUFFER_OVERFLOW        -2
#define RAPI_RESPONSE_TIMEOUT                -1
#define RAPI_RESPONSE_OK                      0
#define RAPI_RESPONSE_NK                      1
#define RAPI_RESPONSE_INVALID_RESPONSE        2
#define RAPI_RESPONSE_CMD_TOO_LONG            3
#define RAPI_RESPONSE_BAD_CHECKSUM            4
#define RAPI_RESPONSE_BAD_SEQUENCE_ID         5
#define RAPI_RESPONSE_ASYNC_EVENT             6
#define RAPI_RESPONSE_FEATURE_NOT_SUPPORTED   7
#define RAPI_RESPONSE_CANCELLED               8 // dropped from the queue unsent (see holdSend())

// Priority commands (see setPriority()) are retried this many extra times on a
// timeout, bad checksum or invalid reply.
#ifndef RAPI_PRIORITY_RETRIES
#define RAPI_PRIORITY_RETRIES 2
#endif

// While sending is held (see holdSend()), at most this many ordinary commands
// may wait in the queue; the rest of it is kept free for priority commands.
static_assert(RAPI_MAX_COMMANDS >= 2, "RAPI_MAX_COMMANDS must leave at least one usable queue slot");

#ifndef RAPI_HOLD_MAX_QUEUED
#define RAPI_HOLD_MAX_QUEUED 3
#endif

// _flags
#define RSF_SEQUENCE_ID_ENABLED   0x01
#define RSF_SEQUENCE_ID_SEEN      0x02 // controller has echoed an id
#define RSF_CMD_IS_VERSION        0x04 // command in flight is $GV

typedef std::function<void()> RapiEventHandler;

/*
 * A completion callback with fixed inline storage.
 *
 * This deliberately does not use std::function. The command queue is a static
 * array that lives for the lifetime of the program, and std::function
 * heap-allocates any callable larger than its small-object buffer (8 bytes
 * here). Every queued command therefore left a small allocation on the heap,
 * re-made at a new address each time a slot was reused, which walks steadily
 * further into the largest free block. On a device that sends RAPI commands
 * continuously the largest contiguous block fell from ~57KB to ~2KB over a
 * couple of days while total free heap barely moved -- fragmentation, not a
 * leak, and eventually fatal to anything needing a large buffer (TLS, OTA).
 *
 * Storing the callable inline removes the allocation entirely. A callable too
 * large to fit is a compile-time error, so the failure mode is a build break
 * rather than a silent return to heap allocation.
 *
 * return values passed to the callback: see RAPI_RESPONSE_XXXX
 */
class RapiCommandCompleteHandler
{
  typedef void (*InvokeFn)(void *, int);
  typedef void (*DestroyFn)(void *);
  typedef void (*CopyFn)(void *, const void *);

  alignas(8) char _storage[RAPI_HANDLER_CAPACITY];
  InvokeFn _invoke;
  DestroyFn _destroy;
  CopyFn _copy;

  void _adopt(const RapiCommandCompleteHandler &other)
  {
    if(nullptr != other._copy) {
      other._copy(_storage, other._storage);
      _invoke = other._invoke;
      _destroy = other._destroy;
      _copy = other._copy;
    }
  }

public:
  RapiCommandCompleteHandler() :
    _storage{}, _invoke(nullptr), _destroy(nullptr), _copy(nullptr) {}
  RapiCommandCompleteHandler(std::nullptr_t) :
    _storage{}, _invoke(nullptr), _destroy(nullptr), _copy(nullptr) {}

  template <typename F,
            typename = typename std::enable_if<
              !std::is_same<typename std::decay<F>::type,
                            RapiCommandCompleteHandler>::value &&
              !std::is_same<typename std::decay<F>::type,
                            std::nullptr_t>::value>::type>
  RapiCommandCompleteHandler(F &&callable) :
    _storage{}, _invoke(nullptr), _destroy(nullptr), _copy(nullptr)
  {
    typedef typename std::decay<F>::type T;
    static_assert(sizeof(T) <= RAPI_HANDLER_CAPACITY,
      "RAPI callback captures too much state; shrink the capture or raise RAPI_HANDLER_CAPACITY");
    static_assert(alignof(T) <= 8, "RAPI callback over-aligned for inline storage");
    new (_storage) T(std::forward<F>(callable));
    _invoke = [](void *p, int result) { (*static_cast<T *>(p))(result); };
    _destroy = [](void *p) { static_cast<T *>(p)->~T(); };
    _copy = [](void *dst, const void *src) { new (dst) T(*static_cast<const T *>(src)); };
  }

  RapiCommandCompleteHandler(const RapiCommandCompleteHandler &other) :
    _storage{}, _invoke(nullptr), _destroy(nullptr), _copy(nullptr)
  {
    _adopt(other);
  }

  RapiCommandCompleteHandler &operator=(const RapiCommandCompleteHandler &other)
  {
    if(this != &other) {
      reset();
      _adopt(other);
    }
    return *this;
  }

  ~RapiCommandCompleteHandler() { reset(); }

  void reset()
  {
    if(nullptr != _destroy) {
      _destroy(_storage);
    }
    _invoke = nullptr;
    _destroy = nullptr;
    _copy = nullptr;
  }

  explicit operator bool() const { return nullptr != _invoke; }
  void operator()(int result) const
  {
    if(nullptr != _invoke) {
      _invoke(const_cast<char *>(_storage), result);
    }
  }

  friend bool operator==(const RapiCommandCompleteHandler &h, std::nullptr_t) { return nullptr == h._invoke; }
  friend bool operator!=(const RapiCommandCompleteHandler &h, std::nullptr_t) { return nullptr != h._invoke; }
  friend bool operator==(std::nullptr_t, const RapiCommandCompleteHandler &h) { return nullptr == h._invoke; }
  friend bool operator!=(std::nullptr_t, const RapiCommandCompleteHandler &h) { return nullptr != h._invoke; }
};

struct CommandItem {
  char command[RAPI_CMD_BUFLEN];
  RapiCommandCompleteHandler handler;
  unsigned int timeout;
  bool priority;
  uint8_t retries;
};

class RapiSender {
private:
  Stream *_stream;
  uint32_t _sent;
  uint32_t _success;
  bool _connected;
  uint8_t _sequenceId;
  uint8_t _flags;
  int _tokenCnt;
  const char *_tokens[RAPI_MAX_TOKENS];
  RapiEventHandler _onRapiEvent;

  Queue<CommandItem> _commandQueue;
  RapiCommandCompleteHandler _completeHandler;
  uint32_t _timeout;
  bool _waitingForReply;
  unsigned long _holdUntil;
  bool _holding;
  size_t _holdMaxQueued;
  bool _priority;
  // the command in flight, kept so a priority command can be re-sent
  CommandItem _inFlight;

  char _respBuf[RAPI_BUFLEN];
  char _respBufOrig[RAPI_BUFLEN];

  int _tokenize();
  void _sendNextCmd();
  void _sendCmd(const char *cmdstr);
  void _sendTail(uint8_t chk, bool addSeq = true);
  int _waitForResult(unsigned long timeout);
  void _commandComplete(int result);
  bool _sendHeld();
  bool _retryInFlight(int result);
  uint8_t _sequenceIdEnabled() {
    return (_flags & RSF_SEQUENCE_ID_ENABLED) ? 1 : 0;
  }
public:

  RapiSender(Stream *stream);

  // Strictly parse a RAPI/firmware version token: exactly three
  // dot-separated groups of 1-3 decimal digits and nothing else, so "2junk",
  // "", "2.0", "2.0.3x" or an overlong number are rejected instead of being
  // read as a partial version. Inline so replacement senders (e.g. a
  // simulator) get the same parser.
  static bool parseVersion(const char *s, int &major, int &minor, int &patch)
  {
    int part[3];
    if(nullptr == s) {
      return false;
    }
    for(int i = 0; i < 3; i++) {
      int digits = 0, val = 0;
      while(*s >= '0' && *s <= '9') {
        if(++digits > 3) {
          return false;
        }
        val = val * 10 + (*s++ - '0');
      }
      if(0 == digits) {
        return false;
      }
      part[i] = val;
      if(i < 2 && '.' != *s++) {
        return false;
      }
    }
    if('\0' != *s) {
      return false;
    }
    major = part[0];
    minor = part[1];
    patch = part[2];
    return true;
  }

  // Usable queue slots (the ring buffer keeps one slot empty).
  static constexpr size_t queueCapacity() { return RAPI_MAX_COMMANDS - 1; }

  // Send nothing to the controller for the next ms milliseconds (e.g. right
  // after it reports a restart). Commands are queued, not dropped, and are
  // sent in order once the hold expires. Calling again restarts the hold.
  // While held, ordinary commands beyond maxQueued (clamped to
  // queueCapacity()) are refused with
  // RAPI_RESPONSE_QUEUE_FULL. Re-arming the hold cancels (RAPI_RESPONSE_CANCELLED)
  // priority commands still queued from before, and stops the one in flight
  // from being retried: they belong to the previous controller session.
  void holdSend(unsigned long ms, size_t maxQueued = RAPI_HOLD_MAX_QUEUED);

  // Commands queued while priority is on may use the whole queue, even during
  // a hold, and are retried up to RAPI_PRIORITY_RETRIES times. Applies to
  // sendCmd() calls made directly between setPriority(true) and
  // setPriority(false), not to ones made later from their callbacks.
  void setPriority(bool on) { _priority = on; }
  void setStream(Stream *stream) { _stream = stream; }
  //  void sendString(const char *str) { dbgprint(str); }

  void sendCmd(const char *cmdstr, RapiCommandCompleteHandler callback=nullptr, unsigned long timeout=RAPI_TIMEOUT_MS);
  void sendCmd(String &cmdstr, RapiCommandCompleteHandler callback=nullptr, unsigned long timeout=RAPI_TIMEOUT_MS);
  void sendCmd(const __FlashStringHelper *cmdstr, RapiCommandCompleteHandler callback=nullptr, unsigned long timeout=RAPI_TIMEOUT_MS);

  int sendCmdSync(const char *cmdstr, unsigned long timeout=RAPI_TIMEOUT_MS);
  int sendCmdSync(String &cmdstr, unsigned long timeout=RAPI_TIMEOUT_MS);
  int sendCmdSync(const __FlashStringHelper *cmdstr, unsigned long timeout=RAPI_TIMEOUT_MS);

  void enableSequenceId(uint8_t tf);
  int8_t getTokenCnt() { return _tokenCnt; }
  const char *getResponse() { return _respBufOrig; }
  // Returns an interior pointer into the response buffer, which the next
  // response overwrites. Callbacks must copy anything they intend to keep;
  // storing the pointer leaves a dangling reference. Returns NULL past the
  // token count.
  const char *getToken(int i) {
    if (i < _tokenCnt) return _tokens[i];
    else return NULL;
  }
  void setOnEvent(RapiEventHandler callback) {
    _onRapiEvent = callback;
  }

  uint32_t getSent() {
    return _sent;
  }
  uint32_t getSuccess() {
    return _success;
  }
  bool isConnected() {
    return _connected;
  }

  void loop();
  bool hasPendingCommands() {
    return !_commandQueue.empty();
  }
  // Drains queued commands and the one in flight, running their completion
  // handlers. Returns true if everything drained, false if `timeout` ms passed
  // first -- in which case the remainder is still queued and still completes
  // normally from loop(); only the synchronous wait is abandoned.
  bool flush(unsigned long timeout = RAPI_FLUSH_TIMEOUT_MS);
};

