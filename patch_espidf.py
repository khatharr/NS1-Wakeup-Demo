"""Apply the ESP-IDF 5.5 Bluedroid SDP array-bound correction before building."""
from pathlib import Path

Import("env")

framework = Path(env.PioPlatform().get_package_dir("framework-espidf"))
source = framework / "components/bt/host/bluedroid/btc/profile/std/sdp/btc_sdp.c"

# Preserve the installed file's line endings and all unrelated bytes.
data = source.read_bytes()
old = b"for (size_t i = 0; i <= SDP_MAX_RECORDS; i++) {"
new = b"for (size_t i = 0; i < SDP_MAX_RECORDS; i++) {"
context = b"if (sdp_local_param.sdp_slots[i] != NULL && sdp_local_param.sdp_slots[i]->state == SDP_RECORD_ALLOCED) {"

# Match the exact loop and its first statement, not other SDP loops.
def matches(loop):
    return [
        i for i, line in enumerate(lines[:-1])
        if line.strip() == loop and lines[i + 1].strip() == context
    ]


lines = data.splitlines(keepends=True)
original = matches(old)
patched = matches(new)
if len(original) == 1 and not patched:
    i = original[0]
    lines[i] = lines[i].replace(old, new, 1)
    source.write_bytes(b"".join(lines))
    print("ESP-IDF: fixed btc_sdp.c SDP_MAX_RECORDS loop bound")
elif not original and len(patched) == 1:
    pass  # Already corrected.
else:
    raise RuntimeError("Unexpected btc_sdp.c source; review SDP loop-bound patch")
