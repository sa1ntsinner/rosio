# Evidence contracts

A diagnostic is conditional on a profile, the facts its collectors could read, and the
semantics of the API it describes. A missing fact does not establish a missing provider.
Successful parsing does not establish that every endpoint in a file was recovered.

## Service and action providers

`service.no_server` requires a known client endpoint, another known server of its wire
type, and a closed provider set in the selected profile. A declared node whose sources
were not read whole leaves that set open. A dynamically named service or action also
leaves it open. In either case the rule abstains; silence is not a claim of correctness.
Known endpoints remain queryable. A known mismatch on an independent channel remains
eligible for its own compatibility rule.

The current closure test uses `read_whole` and is deliberately conservative. It does not
yet expose a predicate-specific completeness certificate, and does not prove that external
code cannot create an endpoint. Extending closure requires a positive control for genuine
absence, a counterexample with an unread provider, and a profile isolation control.

## Wire types

`pkg/msg/Name`, `pkg::msg::Name` and the Python `pkg/Name` spelling identify a ROS type.
A two-part C++ name is a program representation, not evidence of a ROS wire type.
Unresolved custom types do not enter a wire compatibility comparison. This does not
resolve a TypeAdapter mapping: such a mapping needs its own extracted evidence before a
comparison can become definitive.

## Served snapshots

Each MCP tool obtains one validated atlas object before answering. Every subsequent call
checks the current input bytes. Equal bytes reuse the object; changed bytes must parse and
validate before replacing it. Already returned atlas objects remain unchanged by refresh.
Deletion, invalid encoding, malformed JSON and invalid schema refuse the next request.
They never fall back to a cached answer. Restoring a valid file allows subsequent calls.

Writers should replace a complete atlas atomically. An in-place partial write may be
refused while it is incomplete. Reading input bytes on every request favours correctness
over filesystem metadata caching, since timestamps and size alone cannot establish identity.
Large-atlas read cost must be measured before changing this policy.

This contract covers the atlas file, not freshness of its source workspace or runtime.
Source changes, observation windows and externally addressable snapshot handles still need
separate contracts. Refreshing a file does not establish that extraction read current inputs.
