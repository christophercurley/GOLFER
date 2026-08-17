# GOLFER Protocol v1

**Status:** Draft / Unstable  
**Protocol Version:** `1`

GOLFER Protocol defines the host-facing communication interface used to control, configure, query, and receive data from a GOLFER.

Protocol v1 is under active development and may change incompatibly prior to the GOLFER v1 release. Once declared stable, existing v1 wire contracts must not be modified incompatibly.

## Design Principles

- The protocol is **transport-independent**.
- USB CDC is the primary initial transport.
- Messages use explicit binary framing with end-to-end integrity checking.
- Structured payloads normally use Postcard serialization.
- Bulk data may use purpose-built binary payloads where appropriate.
- New functionality should extend Protocol v1 through new services, opcodes, events, and payload types rather than changing existing wire contracts.
- The firmware remains authoritative over device behavior. The protocol requests operations; it does not implement them.
- Loss of asynchronous data must be observable rather than silently hidden.

---

# Frame Format

Every GOLFER message is carried in a frame consisting of:

```text
Core Header
Optional Header Extension
Payload
Frame CRC32
```

Current Protocol v1 transmitters use a 16-byte header with no extension.

All multi-byte integer fields use little-endian byte order.

```text
Offset  Size  Field
------  ----  ------------------------
0       4     Magic
4       1     Protocol Version
5       1     Header Length
6       1     Message Class
7       1     Service
8       1     Opcode
9       1     Status
10      2     Header CRC16
12      2     Message ID
14      2     Payload Length
16      N     Optional Header Extension
H       P     Payload
H+P     4     Frame CRC32
```

Where:

```text
H = Header Length
P = Payload Length
```

Total frame length is:

```text
Header Length + Payload Length + 4
```

The payload always begins at:

```text
offset = Header Length
```

A decoder must not assume that the payload permanently begins at byte 16.

---

# Magic

The four-byte frame magic is:

```text
"GOLF"
```

Hexadecimal:

```text
47 4F 4C 46
```

Magic identifies a candidate beginning of a GOLFER frame.

Magic alone is **not sufficient** to establish synchronization. Arbitrary payload data may legally contain the same byte sequence.

A decoder must validate the Header CRC16 before trusting the candidate header.

---

# Protocol Version

Protocol v1 uses:

```text
0x01
```

`0x00` is reserved.

The protocol version identifies the fundamental framing contract, not the set of features supported by a particular GOLFER.

Adding services, opcodes, events, status codes, or payload types does not by itself require a new protocol version.

Protocol v2 should only be introduced when an incompatible change to the fundamental framing contract cannot reasonably be represented by extending v1.

---

# Header Length

Current Protocol v1 transmitters use:

```text
Header Length = 16
```

or:

```text
0x10
```

Protocol v1 receivers must use Header Length to determine where the payload begins.

Future v1 header extensions may increase Header Length.

Any header extension introduced within Protocol v1 **must be backward-ignorable** by implementations that do not understand it.

A future feature that changes the interpretation of the existing core header or payload in a way older v1 implementations cannot safely ignore requires a new protocol version rather than a v1 header extension.

The Header CRC16 protects the 16-byte core header. The Frame CRC32 protects the complete transmitted header, including any extension bytes.

---

# Header CRC16

Bytes 10–11 contain a CRC16 protecting the fixed 16-byte core header.

Protocol v1 uses:

```text
CRC-16/IBM-3740
commonly known as CRC-16/CCITT-FALSE
```

Parameters:

```text
Width:   16
Poly:    0x1021
Init:    0xFFFF
RefIn:   false
RefOut:  false
XorOut:  0x0000
Check:   0x29B1 for "123456789"
```

When calculating the Header CRC16:

1. Begin with the first 16 bytes of the frame.
2. Treat bytes 10–11, the Header CRC16 field itself, as zero.
3. Calculate CRC16 over those 16 bytes.
4. Store the resulting CRC in bytes 10–11 in little-endian order.

A receiver locating `"GOLF"` in a byte stream must validate this CRC before trusting fields such as Header Length or Payload Length.

This substantially reduces the chance of falsely synchronizing to `"GOLF"` occurring naturally inside payload data.

---

# Frame CRC32

Every GOLFER frame ends with a four-byte CRC32 trailer.

Protocol v1 uses:

```text
CRC-32/ISO-HDLC
```

Parameters:

```text
Width:   32
Poly:    0x04C11DB7
Init:    0xFFFFFFFF
RefIn:   true
RefOut:  true
XorOut:  0xFFFFFFFF
Check:   0xCBF43926 for "123456789"
```

The CRC32 is calculated over:

```text
complete transmitted header
+
complete payload
```

This includes:

- the Header CRC16 field;
- any future header-extension bytes;
- the entire payload.

The four-byte Frame CRC32 trailer itself is not included in the CRC calculation.

The CRC32 value is transmitted little-endian.

A frame with an invalid Frame CRC32 must not be delivered to the application layer.

This end-to-end integrity check is part of GOLFER framing even when the underlying transport already provides integrity protection.

This allows the same protocol to be used safely over future transports that may provide weaker guarantees than USB.

---

# Message Class

The Message Class describes the role played by a frame.

```text
0x01  Request
0x02  Response
0x03  Event
```

## Request

A host sends a Request when asking a GOLFER to perform an operation or return information.

A Request:

- uses a nonzero Message ID;
- uses `Status = 0`;
- may contain an opcode-specific payload.

## Response

A GOLFER sends a Response after processing a Request.

A Response:

- uses the same Message ID as the corresponding Request;
- retains the Service and Opcode of the Request;
- uses Status to indicate the broad result of the operation;
- may contain a response or error-detail payload.

## Event

A GOLFER may emit an Event without a corresponding host Request.

Events are intended for asynchronous activity such as:

- received LoRa packets;
- GPS position updates;
- log records;
- other streaming or notification data.

For Events, Message ID acts as an **event sequence number**.

---

# Message ID

Message ID is an unsigned 16-bit integer whose meaning depends on Message Class.

## Requests

The host assigns a nonzero Message ID:

```text
0x0001–0xFFFF
```

## Responses

A Response copies the Message ID from its corresponding Request.

## Events

For Events, Message ID is a monotonically increasing sequence number.

The sequence is scoped to an individual event stream identified by:

```text
Service + Opcode
```

For example, these are independent streams:

```text
LoRa / PacketReceived
GPS   / PositionUpdated
Logs  / LogRecord
```

Event sequence values use:

```text
0x0001–0xFFFF
```

and wrap from:

```text
0xFFFF → 0x0001
```

`0x0000` is reserved.

An event sequence number must advance when an event is produced for delivery, **before** enqueueing or transmission.

Therefore, if firmware drops an event because of back-pressure or queue exhaustion, the next successfully delivered Event exposes the loss through a sequence gap.

Example:

```text
Event 104
Event 105
Event 107
```

The host can determine that Event 106 was lost.

---

# Event Delivery and Back-Pressure

Unless a specific service explicitly defines stronger delivery guarantees, Events are:

```text
best-effort
```

Event delivery must not block time-critical firmware subsystems.

A slow, stalled, or disconnected host must not cause components such as:

- LoRa receive processing;
- GPS acquisition;
- timing-sensitive firmware tasks

to block indefinitely while waiting for USB or another transport.

When buffering is exhausted, firmware may drop Events.

Such loss must remain observable through the event sequence-number mechanism.

Operations where loss is unacceptable, such as file transfer, must use explicit transactional or flow-control semantics rather than relying on best-effort Events.

---

# Services

A Service identifies the GOLFER subsystem to which an operation belongs.

The initial namespace is expected to include:

```text
System
LoRa
GPS
Files
Logs
```

Numeric Service identifiers are assigned only when the corresponding service is implemented.

Unknown Service identifiers do not make a frame malformed.

A structurally valid Request addressed to an unsupported Service receives:

```text
UnsupportedService
```

---

# Opcodes

An Opcode identifies an operation or event within a Service.

Opcode namespaces are scoped by:

```text
Message Class + Service
```

Therefore:

```text
Request / LoRa / 0x01
```

and:

```text
Event / LoRa / 0x01
```

are distinct protocol definitions.

The initial System Request namespace will include operations equivalent to:

```text
GET_INFO
SET_NAME
```

Once Protocol v1 is stable, an assigned opcode must never later be given a different meaning.

If an existing operation eventually requires an incompatible wire contract, a new opcode must be introduced rather than changing the existing opcode.

---

# Status

Status gives the broad result of processing a Request.

Requests and Events always use:

```text
Status = 0
```

Responses use a `StatusCode`.

Initial StatusCode assignments are:

```text
0x00  Ok

0x01  UnsupportedVersion
0x02  UnsupportedService
0x03  UnsupportedOpcode

0x04  InvalidRequest
0x05  InvalidPayload
0x06  InvalidArgument
0x07  InvalidState

0x08  Busy
0x09  NotFound
0x0A  AccessDenied

0x0B  Timeout
0x0C  IoError
0x0D  InternalError
```

Remaining values are reserved.

Assigned numeric values must never later be reused for a different meaning.

## Detailed Errors

Status codes intentionally remain generic.

Service- or operation-specific failure information belongs in the Response payload.

For example:

```text
SYSTEM / SET_NAME / Response

Status:
    InvalidArgument

Payload:
    NameTooLong
    maximum length = 32
```

The Status answers:

> What broad class of result occurred?

The payload may answer:

> Why, specifically, did it occur?

Human-facing software such as `golfer-cli` should construct error messages from structured protocol information rather than parsing diagnostic strings emitted by firmware.

---

# Payload Length

Payload Length is an unsigned 16-bit integer specifying the number of payload bytes following the complete header.

The framing format can represent payloads from:

```text
0–65,535 bytes
```

This is a **protocol encoding limit**, not a guarantee that every GOLFER can accept a 65,535-byte payload.

Individual GOLFER implementations advertise their actual maximum receive payload through System information.

A host must not transmit a payload larger than the target device's advertised limit.

Receivers should not assume that the complete declared payload must be buffered in RAM at once.

For example, an oversized or bulk frame may be consumed, CRC-checked, or discarded incrementally.

Large objects such as files are transferred using bounded chunks rather than a single enormous frame.

---

# Device Receive Capability

`GET_INFO` includes:

```text
max_rx_payload
```

This value is the maximum payload, in bytes, that the device agrees to accept in a single inbound GOLFER frame.

It must not exceed:

```text
65,535
```

Example:

```text
Protocol maximum:     65,535 bytes
Device max_rx_payload: 4,096 bytes
```

The host must use the device's advertised limit.

This allows GOLFER implementations with different memory resources to share the same protocol.

---

# Payload Encoding

Each combination of:

```text
Message Class + Service + Opcode
```

defines the expected payload contract.

## Structured Payloads

Structured payloads normally use Postcard serialization.

Protocol wire types are defined in the shared `golfer-protocol` crate.

Postcard does **not** provide field-tag-based forward compatibility.

Therefore, once a payload schema is stable, the following changes are incompatible wire changes:

- adding a field;
- removing a field;
- reordering fields;
- changing a field's type;
- otherwise changing its serialized structure.

After Protocol v1 is declared stable, an existing Postcard payload contract **must not be modified in place**.

An incompatible evolution requires a new opcode or explicitly new wire contract.

Protocol structs are wire contracts, not general-purpose application structs.

## Empty Payloads

Operations requiring no request data use a Payload Length of zero.

For example:

```text
System / GET_INFO / Request
Payload Length = 0
```

## Bulk Data

Postcard is not mandatory for bulk byte streams.

Operations such as:

- file transfer;
- raw LoRa payload transfer;
- other high-volume binary streams

may define purpose-built payload layouts.

Bulk transfer protocols should use bounded chunks and explicit transfer semantics.

---

# Stream Transport Behavior

USB CDC is a byte-stream transport.

A transport read is **not** assumed to correspond to one GOLFER frame.

A receiver may observe:

- part of a header;
- one complete frame;
- multiple complete frames;
- one complete frame followed by part of another.

A decoder therefore accumulates or incrementally processes bytes until complete frames become available.

A conceptual decode sequence is:

```text
Search for "GOLF"
        ↓
Read candidate 16-byte core header
        ↓
Validate Header CRC16
        ↓
Validate core header fields
        ↓
Read Header Length
        ↓
Consume any header-extension bytes
        ↓
Read Payload Length
        ↓
Consume payload
        ↓
Read Frame CRC32
        ↓
Validate complete frame CRC
        ↓
Deliver frame
```

If a candidate `"GOLF"` sequence fails Header CRC validation, the decoder resumes searching for another candidate magic sequence.

If the complete frame fails CRC32 validation, the frame is discarded and must not reach the application layer.

---

# Malformed Frames

A decoder must distinguish between:

```text
Malformed frame
```

and:

```text
Valid frame requesting unsupported functionality
```

Examples of malformed requests include violations such as:

- invalid Header CRC16;
- invalid Frame CRC32;
- invalid Message Class value;
- Request with `Message ID = 0`;
- Request with nonzero Status;
- Event with nonzero Status;
- impossible or invalid Header Length;
- payload that violates the expected wire format.

Unknown Service or Opcode values are **not malformed framing**.

They receive normal Responses using:

```text
UnsupportedService
```

or:

```text
UnsupportedOpcode
```

when a trustworthy Request can be identified.

Frames too malformed to provide trustworthy Request information are discarded without a Response.

---

# Compatibility and Versioning

Protocol v1 is intended to remain extensible for the lifetime of the GOLFER v1 protocol family.

The following do **not** require a new protocol version:

- adding a Service;
- adding an Opcode;
- adding an Event;
- adding a StatusCode;
- adding a new payload type;
- adding a replacement operation under a new Opcode;
- adding a backward-ignorable header extension.

Protocol v2 should only be introduced when an incompatible change to the fundamental framing contract cannot reasonably be represented by extending v1.

Before GOLFER v1 is publicly released, Protocol v1 remains unstable and may be changed freely as implementation experience reveals problems.

After Protocol v1 is declared stable:

> **Extend v1; do not mutate v1.**

Assigned numeric meanings must not be recycled.

Published stable payload contracts must not be modified incompatibly.

---

# Security Model

Protocol v1 does not initially provide USB authentication or encryption.

The initial trust model is:

> Physical access to a GOLFER USB connection implies control authority unless firmware policy states otherwise.

Security remains a firmware concern rather than a framing concern.

The protocol is designed so future firmware may impose authentication or authorization policies without changing the meaning of existing operations.

For example, a future authenticated GOLFER may receive:

```text
System / SET_NAME / Request
```

and respond:

```text
Status = AccessDenied
```

when the current session is not authorized.

The raw LoRa interface is intentionally raw.

GOLFER does not implicitly encrypt radio payloads submitted through the LoRa service.

Higher-level applications may independently define authenticated or encrypted radio communications where confidentiality or authenticity is required.

---

# Initial System Service

The first Protocol v1 implementation will provide two System operations.

## GET_INFO

Returns GOLFER system information including at minimum:

- canonical hardware-derived System ID;
- human-readable device name;
- firmware version;
- protocol version;
- configuration schema version;
- maximum accepted receive payload.

The System ID is immutable.

The device name is user-configurable.

The maximum accepted receive payload is exposed as:

```text
max_rx_payload
```

and describes a runtime protocol capability rather than the theoretical 16-bit framing maximum.

## SET_NAME

Requests that firmware update the human-readable device name.

The protocol layer does not directly modify persistent storage.

Firmware validates the request and invokes the appropriate System configuration functionality.

A successful rename persists across reboot.

---

# Device Identity and Discovery

Every physical GOLFER has one canonical System ID derived from the RP2350 hardware identity.

Human-readable names are not unique identifiers.

Multiple GOLFERs may have identical names.

Host software may therefore permit convenient selection by name, but canonical identification must ultimately use the immutable System ID.

USB discovery details are defined separately from the core framing protocol.

---

# Current Development Status

Protocol v1 is currently:

```text
DRAFT / UNSTABLE
```

The first implementation milestone is:

```text
golfer-cli
    ↓
USB CDC
    ↓
GOLFER Protocol v1
    ↓
GET_INFO
SET_NAME
    ↓
persistent System configuration
```

The framing and protocol will continue to be exercised and revised during GOLFER development before the v1 contract is frozen.