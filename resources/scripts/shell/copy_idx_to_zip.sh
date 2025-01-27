#!/bin/bash

# Resolve the script's directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_FOLDER="$(cd "$SCRIPT_DIR/../../profiles" && pwd)"
ZIP_ARCHIVE="vendor_indices.zip"

# Load FTP configuration
CONFIG_FILE="$SCRIPT_DIR/ftp_config"

if [ ! -f "$CONFIG_FILE" ]; then
  echo "Error: Configuration file 'ftp_config' not found in $SCRIPT_DIR."
  exit 1
fi

source "$CONFIG_FILE"

# Check if all required FTP variables are set
if [ -z "$FTP_SERVER" ] || [ -z "$FTP_PORT" ] || [ -z "$FTP_USER" ] || [ -z "$FTP_PASSWORD" ] || [ -z "$REMOTE_PATH" ]; then
  echo "Error: Missing FTP configuration details in 'ftp_config'."
  exit 1
fi

# Check if the source folder exists
if [ ! -d "$SOURCE_FOLDER" ]; then
  echo "Error: Source folder '$SOURCE_FOLDER' does not exist."
  exit 1
fi

cd "$SOURCE_FOLDER" || { echo "Failed to navigate to folder."; exit 1; }

# Create the zip archive
echo "Creating zip archive: $ZIP_ARCHIVE"
rm -f "$ZIP_ARCHIVE"

if ! 7z a -tzip "$ZIP_ARCHIVE" *.idx; then
  echo "No .idx files found or failed to create zip archive."
  exit 1
fi

echo "Zip archive '$ZIP_ARCHIVE' created successfully."

# Upload the zip file to the FTP server
FTP_URL="${FTP_SERVER}:${FTP_PORT}/${REMOTE_PATH}"
echo "Uploading $ZIP_ARCHIVE to $FTP_URL"
curl -T "$ZIP_ARCHIVE" "$FTP_URL" --user "$FTP_USER:$FTP_PASSWORD"

# Check if the upload was successful
if [ $? -eq 0 ]; then
  echo "File uploaded successfully to $REMOTE_PATH!"
else
  echo "Failed to upload file to $REMOTE_PATH."
  exit 1
fi

exit 0
