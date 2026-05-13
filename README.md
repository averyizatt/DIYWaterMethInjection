# Water-Meth Module (external repo placeholder)

Add your water-meth firmware repository at this path (submodule or vendored copy) so it remains an independent PlatformIO project.

Upstream repo:

- `https://github.com/averyizatt/DIYWaterMethInjection`

Expected behavior:
- It uses the shared CAN contract in `shared/can_contract/include/can_contract/can_protocol.h`.
- It publishes/consumes water-meth CAN IDs from the shared contract.
- It is built/flashed with its own PlatformIO environments inside that repo.
