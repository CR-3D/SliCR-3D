#!/bin/bash

# Load SFTP credentials
source "$(dirname "$0")/ftp_credentials.conf"

# Ensure SFTP_PORT is set (default to 22 if empty)
SFTP_PORT=${SFTP_PORT:-22}

# Get script directory dynamically
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESOURCE_DIR="$SCRIPT_DIR/../../profiles"  # Profiles directory
LOCAL_TMP_DIR="/tmp/slcr_update"

# Ensure local temp directory exists
mkdir -p "$LOCAL_TMP_DIR"

# Find staged .ini and .idx files in Git
STAGED_FILES=$(git diff --cached --name-only | grep "^resources/profiles/.*\.\(ini\|idx\)$")

# Check if any files are staged
if [[ -z "$STAGED_FILES" ]]; then
    echo "No .ini or .idx files staged. Skipping SFTP upload."
    exit 0
fi

# Extract filenames (without path)
INI_FILE=""
IDX_FILE=""
for FILE in $STAGED_FILES; do
    BASENAME=$(basename "$FILE")
    if [[ "$BASENAME" == *.ini ]]; then
        INI_FILE="$BASENAME"
    elif [[ "$BASENAME" == *.idx ]]; then
        IDX_FILE="$BASENAME"
    fi
done

# Validate .ini and .idx files exist
if [[ -z "$INI_FILE" ]]; then
    echo "Error: No .ini file staged!"
    exit 1
fi
if [[ -z "$IDX_FILE" ]]; then
    echo "Error: No .idx file staged!"
    exit 1
fi

# Read config_version from the staged .ini file
CONFIG_VERSION=$(awk -F ' = ' '/^config_version/ {print $2}' "$RESOURCE_DIR/$INI_FILE")

# Validate config_version
if [[ -z "$CONFIG_VERSION" ]]; then
    echo "Error: Could not extract config_version from $INI_FILE"
    exit 1
fi

# Rename the .ini file based on config_version
NEW_INI_FILENAME="${CONFIG_VERSION}.ini"
cp "$RESOURCE_DIR/$INI_FILE" "$LOCAL_TMP_DIR/$NEW_INI_FILENAME"

# Compress .idx file into vendor_indices.zip
zip -j "$LOCAL_TMP_DIR/vendor_indices.zip" "$RESOURCE_DIR/$IDX_FILE"

# Ensure ZIP file was created
if [[ ! -f "$LOCAL_TMP_DIR/vendor_indices.zip" ]]; then
    echo "Error: vendor_indices.zip was not created!"
    exit 1
fi

# Define correct remote SFTP paths based on filename
FILENAME_NO_EXT="${INI_FILE%.*}"  # Extract the filename without extension
SFTP_BASE_DIR="/updates/SliCR-3D/v1/repos/cr3d-fff"
SFTP_CR3D_DIR="$SFTP_BASE_DIR/$FILENAME_NO_EXT"  # Use the filename as the SFTP directory

echo "Uploading files to: $SFTP_CR3D_DIR on port $SFTP_PORT"

# Upload files via SFTP
if [[ -n "$SFTP_PASS" ]]; then
    # Password-based authentication using sshpass
    sshpass -p "$SFTP_PASS" sftp -oPort=$SFTP_PORT "$SFTP_USER@$SFTP_HOST" <<EOF
mkdir -p $SFTP_CR3D_DIR
cd $SFTP_CR3D_DIR
put "$LOCAL_TMP_DIR/vendor_indices.zip"
put "$LOCAL_TMP_DIR/$NEW_INI_FILENAME"
bye
EOF
else
    # Key-based authentication (no password needed)
    sftp -oPort=$SFTP_PORT "$SFTP_USER@$SFTP_HOST" <<EOF
mkdir -p $SFTP_CR3D_DIR
cd $SFTP_CR3D_DIR
put "$LOCAL_TMP_DIR/vendor_indices.zip"
put "$LOCAL_TMP_DIR/$NEW_INI_FILENAME"
bye
EOF
fi

# Cleanup
rm -rf "$LOCAL_TMP_DIR"

echo "SFTP update completed successfully!"
