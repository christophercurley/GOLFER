# GOLFER Protocol v1

**Status:** Draft / Unstable  
**Protocol Version:** `1`

GOLFER Protocol defines the host-facing communication interface used to control, configure, query, and receive data from a GOLFER.

Protocol v1 is under active development and may change incompatibly prior to the GOLFER v1 release. Once declared stable, existing v1 wire contracts must not be modified incompatibly.

## Design Principles

- The protocol is **transport-independent**.
- USB CDC is the primary initial transport.
- Messages use explicit binary framing.
- Structured payloads normally use Postcard serialization.
- Bulk data may use purpose-built binary payloads where appropriate.
- New functionality should extend Protocol v1 through new services, opcodes, and payload types rather than changing existing wire contracts.
- The firmware remains authoritative over device behavior. The protocol requests operations; it does not implement them.

## Frame Format

Every GOLFER message is carried in a frame consisting of a fixed 16-byte header followed by an optional payload.

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
10      2     Reserved
12      2     Request ID
14      2     Payload Length
16      N     Payload
```

### Magic

The four-byte frame magic is:

```text
"GOLF"
```

Hexadecimal:

```text
47 4F 4C 46
```

The magic value identifies the beginning of a GOLFER frame and permits a stream decoder to recover framing after receiving incomplete or invalid data.

### Protocol Version

Protocol v1 uses:

```text
0x01
```

`0x00` is reserved.

The protocol version identifies the fundamental framing contract, not the set of features supported by a particular GOLFER.

Adding services, opcodes, events, status codes, or other compatible extensions does not require a new protocol version.

### Header Length

Protocol v1 headers are:

```text
16 bytes
```

Therefore:

```text
Header Length = 0x10
```

The field exists to permit future framing extensions without changing the location of the core v1 header fields.

### Message Class

The message class describes the role played by a frame.

```text
0x01  Request
0x02  Response
0x03  Event
```

#### Request

A host sends a Request when asking a GOLFER to perform an operation or return information.

A Request:

- must use a nonzero Request ID;
- must use `Status = 0`;
- may contain an opcode-specific payload.

#### Response

A GOLFER sends a Response after processing a Request.

A Response:

- must use the same Request ID as the corresponding Request;
- retains the Service and Opcode of the Request;
- uses the Status field to indicate the broad result of the operation;
- may contain a response or error-detail payload.

#### Event

A GOLFER may emit an Event without a corresponding host Request.

Events are intended for asynchronous activity such as:

- received LoRa packets;
- GPS position updates;
- log records;
- other future streaming or notification data.

An Event:

- uses `Request ID = 0`;
- uses `Status = 0`.

## Services

A Service identifies the GOLFER subsystem to which an operation belongs.

The initial namespace is expected to include:

```text
System
LoRa
GPS
Files
Logs
```

Numeric service identifiers are assigned only when the corresponding service is implemented.

Unknown service identifiers do not make a frame malformed. A valid Request addressed to an unsupported service receives an `UnsupportedService` Response.

## Opcodes

An Opcode identifies an operation within a Service.

For example, the initial System service will include operations equivalent to:

```text
GET_INFO
SET_NAME
```

Opcode values are scoped to their Service.

A valid Request using an unknown opcode receives an `UnsupportedOpcode` Response.

Once Protocol v1 is stable, an assigned opcode must never be given a different meaning.

If an existing operation eventually requires an incompatible wire contract, a new opcode should be introduced rather than changing the existing opcode.

## Status

The Status field gives the broad result of processing a Request.

Requests and Events always use:

```text
Status = 0
```

Responses use a `StatusCode`.

Initial StatusCode assignments:

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

### Detailed Errors

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

Human-readable software such as `golfer-cli` should construct error messages from structured protocol information rather than parsing diagnostic strings produced by firmware.

## Request ID

Request IDs are unsigned 16-bit integers.

```text
0x0000       Reserved for messages not associated with a Request
0x0001-FFFF  Host Request IDs
```

A Response must contain the same Request ID as its originating Request.

This permits asynchronous Events and multiple outstanding operations to coexist without ambiguity.

Request IDs may wrap after `0xFFFF`.

## Payload Length

Payload Length is an unsigned 16-bit integer specifying the number of payload bytes following the header.

Maximum payload size for one frame:

```text
65,535 bytes
```

Large objects such as files are transferred using multiple bounded chunks rather than a single enormous frame.

## Payload Encoding

Each combination of:

```text
Message Class + Service + Opcode
```

defines the expected payload type.

### Structured Payloads

Structured payloads normally use Postcard serialization.

Protocol wire types are defined in the shared `golfer-protocol` crate.

Wire types must be treated as protocol contracts rather than convenient application data structures.

Once Protocol v1 is declared stable, an existing published payload schema must not be modified incompatibly.

If incompatible evolution is necessary, a new opcode or explicitly new payload contract should be introduced.

### Empty Payloads

Operations requiring no data use a payload length of zero.

For example:

```text
SYSTEM / GET_INFO / Request
Payload Length = 0
```

### Bulk Data

Postcard is not mandatory for bulk byte streams.

Operations such as:

- file transfer;
- raw LoRa payload transfer;
- other high-volume binary streams

may define purpose-built payload layouts.

Bulk transfer protocols should use bounded chunks.

## Stream Transport Behavior

USB CDC is a byte-stream transport.

A transport read is **not** assumed to correspond to one GOLFER frame.

A receiver may observe:

- part of a header;
- one complete frame;
- multiple complete frames;
- one complete frame followed by part of another.

A decoder therefore accumulates bytes until at least one complete frame is available.

Basic decoding sequence:

```text
Locate "GOLF"
    ↓
Read 16-byte header
    ↓
Validate header
    ↓
Read Payload Length
    ↓
Wait until complete payload is available
    ↓
Decode frame
```

Malformed or untrustworthy input is discarded and the decoder attempts to resynchronize using the next valid `GOLF` magic sequence.

## Reserved Field

Bytes 10–11 of the v1 header are reserved.

They must be transmitted as:

```text
0x0000
```

No flags are currently defined.

Frame-level features should not be assigned speculatively. The reserved space remains available during Protocol v1 development if a genuinely general framing requirement emerges.

## Compatibility and Versioning

Protocol v1 is intended to remain extensible for the lifetime of the GOLFER v1 protocol family.

The following do **not** require a new protocol version:

- adding a Service;
- adding an Opcode;
- adding an Event;
- adding a StatusCode;
- adding a new payload type;
- adding a new revision of an operation using a new Opcode.

Unknown Services and Opcodes are handled through normal Response status codes.

Protocol v2 should only be introduced when an incompatible change to the fundamental framing contract cannot reasonably be represented by extending v1.

Before GOLFER v1 is publicly released, Protocol v1 remains unstable and may be changed freely as implementation experience reveals problems.

After Protocol v1 is declared stable:

> Extend v1; do not mutate v1.

## Security Model

Protocol v1 does not initially provide USB authentication or encryption.

The initial trust model is:

> Physical access to a GOLFER USB connection implies control authority unless firmware policy states otherwise.

Security remains a firmware concern rather than a framing concern.

The protocol is designed so that future firmware may impose authentication or authorization policies without changing the meaning of existing operations.

For example, a future authenticated GOLFER may receive:

```text
SYSTEM / SET_NAME / Request
```

and respond:

```text
Status = AccessDenied
```

when the current session is not authorized.

The raw LoRa interface is intentionally raw. GOLFER does not implicitly encrypt radio payloads submitted through the LoRa service.

Higher-level GOLFER radio applications may independently define authenticated or encrypted communications where privacy or authenticity is required.

## Initial System Service

The first Protocol v1 implementation will provide two System operations.

### GET_INFO

Returns GOLFER system information, including at minimum:

- canonical hardware-derived System ID;
- human-readable device name;
- firmware version;
- protocol version;
- configuration schema version.

The System ID is immutable.

The device name is user-configurable.

### SET_NAME

Requests that firmware update the human-readable device name.

The protocol layer does not directly modify persistent storage.

Firmware validates the request and invokes the appropriate System configuration functionality.

A successful rename persists across reboot.

## Device Identity and Discovery

Every physical GOLFER has one canonical System ID derived from the RP2350 hardware identity.

Human-readable names are not unique identifiers.

Multiple GOLFERs may have identical names.

Host software may therefore permit convenient selection by name, but canonical identification must ultimately use the immutable System ID.

USB discovery details are defined separately from the core framing protocol.

## Current Development Status

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