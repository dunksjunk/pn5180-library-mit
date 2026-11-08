# Security policy

## Supported versions

| Version | Supported |
|---|---|
| 0.1.x | yes |
| older | no |

Before 1.0 only the latest 0.x release receives fixes.

## Reporting a vulnerability

Please **do not open a public issue** for a security problem.

Report it privately through GitHub: open the repository's **Security** tab
and choose **Report a vulnerability** (GitHub private vulnerability
reporting). If that button is not available, open a regular issue that
only asks for a private contact, without any details of the problem, and
the maintainer will get back to you.

Include the affected version or commit, the platform (board, HAL), and a
minimal way to reproduce it, e.g. the card response or frame that
triggers it.

This is a single-maintainer project, so handling is best effort. You will
get an acknowledgement, and a fix or an assessment, as soon as practical.
Please allow time for a fix before you disclose the issue publicly.

## Scope

In scope: the driver code in `src/`, the examples in `examples/`, and the
CI and tooling in this repository. Examples:

- memory-safety bugs: a malformed card answer, NDEF message or TLV that
  makes the parser read or write outside its buffers;
- unbounded loops or hangs caused by card data;
- writes outside the documented area of a tag, e.g. touching Type 2 Tag
  pages 0-3, lock or configuration bytes.

The parsers are tested with malformed input, and CI runs the host tests
under AddressSanitizer and UndefinedBehaviorSanitizer.

## What this library does not protect against

The library implements the transport and tag-memory layers of ISO/IEC 14443A,
ISO/IEC 15693 and NFC Forum Type 2 / Type 5 Tags. It has **no cryptographic
authentication**:

- **A card UID is not a secret** and is not proof of identity. UIDs can be
  read by anyone in range, and some cards can be cloned or emulated with any
  UID. Do not use a UID alone for access control.
- NDEF content on unlocked tags can be rewritten by anyone in range.
- NFC itself does not prevent relay or eavesdropping attacks.

Design your application with these limits in mind. Such reports are about
NFC in general, not vulnerabilities in this library.
