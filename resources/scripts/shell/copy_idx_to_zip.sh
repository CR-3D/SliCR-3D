#!/bin/bash

# Resolve the script's directory and set the folder and zip archive paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_FOLDER="$(cd "$SCRIPT_DIR/../../profiles" && pwd)"
ZIP_ARCHIVE="vendor_indices.zip"

# Check if the source folder exists
if [ ! -d "$SOURCE_FOLDER" ]; then
  echo "Error: Source folder '$SOURCE_FOLDER' does not exist."
  exit 1
fi

# Navigate to the source folder
cd "$SOURCE_FOLDER" || { echo "Failed to navigate to folder."; exit 1; }

# Remove existing zip file if it exists
echo "Creating zip archive: $ZIP_ARCHIVE"
rm -f "$ZIP_ARCHIVE"

# Use 7z to create a .zip archive
if ! 7z a -tzip "$ZIP_ARCHIVE" *.idx; then
  echo "No .idx files found or failed to create zip archive."
  exit 1
fi

echo "Zip archive '$ZIP_ARCHIVE' created successfully."
exit 0
