README_versioning.txt
---------------------

Firmware Versioning Guide
========================

This project uses a versioning system with the following files:
- LogAndStream/build/version.txt
- LogAndStream/build/version_base.txt
- LogAndStream/Shimmer_Driver/version.h

How to Increment the Version
----------------------------

1. Open a terminal (Git Bash recommended on Windows).
2. Navigate to the project directory:
   cd LogAndStream/build

3. Run the version increment script:
   ./scripts/increment_version.sh [major|minor|patch]

   - If no argument is given, 'patch' is incremented by default.
   - Examples:
     ./scripts/increment_version.sh patch   # Increments patch (e.g., 1.00.030 -> 1.00.031)
     ./scripts/increment_version.sh minor   # Increments minor (e.g., 1.00.030 -> 1.01.000)
     ./scripts/increment_version.sh major   # Increments major (e.g., 1.00.030 -> 2.00.000)

4. The script will:
   - Update `version.txt` with the new version.
   - Regenerate `version.h` with the new version numbers.
   - Print the updated version to the terminal.

5. Commit the changes:
   git add build/version.txt ../Shimmer_Driver/version.h
   git commit -m "chore: bump firmware version"
   git push

Useful Notes
------------

- The script enforces version format and value limits (major: 0-65535, minor/patch: 0-255).
- If the version files do not exist, they will be created with default values.
- The version struct is embedded in the firmware binary for traceability.
- Always ensure your working tree is clean before running the script and committing.
- The script is compatible with Git Bash on Windows.

Troubleshooting
---------------

- If you get a permissions error, run: chmod +x scripts/increment_version.sh
- If you see format errors, check that `version.txt` and `version_base.txt` match the expected formats:
  - version_base.txt: MAJOR.MINOR (e.g., 1.00)
  - version.txt: MAJOR.MINOR.PATCH (e.g., 1.00.030)

For more details, see the comments in `increment_version.sh`.
