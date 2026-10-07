#!/usr/bin/env python3
"""fs_new_key_consistency.py - verify every new fs_* key is wired in all the
places the fork requires, and that the localization catalog carries its strings.

The fork has four wiring points for a printer-owned fs_ key plus the .pot, and a
key missing from any one of them fails in a different, confusing way:

  PrintConfig.hpp        the class member exists at all (missing -> C++ no member)
  PrintConfig.cpp        the option is registered (missing -> key silently ignored)
  Preset.cpp             printer-preset membership (missing -> not saved with the printer)
  GCode.cpp dump ban     config-dump stability (missing -> every preset diffs)
  OrcaSlicer.pot         label + tooltip msgids (missing -> untranslated UI)
  roundtrip test         the key is covered by the 3MF round trip

Run from the repository root. Exit 1 on any gap.
"""

import re
import sys

NEW_KEYS = [
    "fs_tool_preheat_lead_s",
    "fs_fiber_tail_margin_mm",
    "fs_fiber_release_length_mm",
    "fs_fiber_release_speed_mm_s",
    "fs_fiber_release_anchor_mm",
    "fs_restart_feed_rate",
]

FILES = {
    "hpp": "src/libslic3r/PrintConfig.hpp",
    "cpp": "src/libslic3r/PrintConfig.cpp",
    "preset": "src/libslic3r/Preset.cpp",
    "gcode": "src/libslic3r/GCode.cpp",
    "pot": "localization/i18n/OrcaSlicer.pot",
    "roundtrip": "tests/libslic3r/test_fiber_3mf_roundtrip.cpp",
}


def read(path):
    with open(path, encoding="utf-8", errors="replace") as fh:
        return fh.read()


def main():
    texts = {k: read(v) for k, v in FILES.items()}
    failures = 0

    for key in NEW_KEYS:
        checks = [
            ("PrintConfig.hpp member",
             re.search(r"\(\(ConfigOption\w+,\s*" + key + r"\)\)", texts["hpp"])),
            ("PrintConfig.cpp registered",
             'this->add("' + key + '"' in texts["cpp"]),
            ("Preset.cpp printer group",
             '"' + key + '"' in texts["preset"]),
            ("GCode.cpp dump ban",
             '"' + key + '"sv' in texts["gcode"]),
            ("roundtrip covered",
             '"' + key + '"' in texts["roundtrip"]),
        ]
        for label, ok in checks:
            if not ok:
                print("MISSING  %-30s %s" % (key, label))
                failures += 1

    # Every label and tooltip string introduced for the new keys must have a
    # msgid. Pull the strings straight out of the PrintConfig.cpp block.
    block = texts["cpp"]
    start = block.find('this->add("' + NEW_KEYS[0] + '"')
    if start == -1:
        print("CANNOT LOCATE the new-key block in PrintConfig.cpp")
        return 1
    end = block.find('this->add("fs_aux_fans_on_toolchange"', start)
    chunk = block[start:end if end != -1 else len(block)]
    strings = re.findall(r'(?:label|tooltip) = L\("((?:[^"\\]|\\.)+)"\)', chunk)
    for s in strings:
        needle = 'msgid "' + s.replace('\\"', '"') + '"'
        if needle not in texts["pot"]:
            print("MISSING  .pot msgid for: %.60s..." % s)
            failures += 1

    print("keys checked: %d   strings checked: %d" % (len(NEW_KEYS), len(strings)))
    print("CONSISTENCY: %s" % ("FAIL" if failures else "PASS"))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
