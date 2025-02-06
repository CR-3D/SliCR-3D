#!/bin/bash

# Load SFTP credentials
source "$(dirname "$0")/ftp_credentials.conf"

# Get script directory dynamically
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Define paths dynamically
RESOURCE_DIR="$SCRIPT_DIR/../../profiles"  # Profiles are in resources/profiles
INI_FILE="$RESOURCE_DIR/CR3D.ini"
IDX_FILE="$RESOURCE_DIR/CR3D.idx"  # Adjust as needed
LOCAL_TMP_DIR="/tmp/slcr_update"

# Define correct remote SFTP paths
SFTP_BASE_DIR="/updates/SliCR-3D/v1/repos/cr3d-fff"
SFTP_CR3D_DIR="$SFTP_BASE_DIR/CR3D"

# Ensure local temp directory exists
mkdir -p "$LOCAL_TMP_DIR"

# Read config_version from the .ini file
CONFIG_VERSION=$(awk -F ' = ' '/^config_version/ {print $2}' "$INI_FILE")

# Validate config_version
if [[ -z "$CONFIG_VERSION" ]]; then
    echo "Error: Could not extract config_version from CR3D.ini"
    exit 1
fi

# Rename the ini file based on config_version
NEW_INI_FILENAME="${CONFIG_VERSION}.ini"
cp "$INI_FILE" "$LOCAL_TMP_DIR/$NEW_INI_FILENAME"

# Ensure IDX file exists
if [[ ! -f "$IDX_FILE" ]]; then
    echo "Error: CR3D.idx file not found at $IDX_FILE!"
    exit 1
fi

# Compress CR3D.idx into vendor_indices.zip
zip -j "$LOCAL_TMP_DIR/vendor_indices.zip" "$IDX_FILE"

# Ensure ZIP file was created
if [[ ! -f "$LOCAL_TMP_DIR/vendor_indices.zip" ]]; then
    echo "Error: vendor_indices.zip was not created!"
    exit 1
fi

# Upload files via SFTP
if [[ -n "$SFTP_PASS" ]]; then
    # Password-based authentication using sshpass
    sshpass -p "$SFTP_PASS" sftp -oPort=$SFTP_PORT "$SFTP_USER@$SFTP_HOST" <<EOF
cd $SFTP_BASE_DIR
mkdir -p CR3D
put "$LOCAL_TMP_DIR/vendor_indices.zip"
cd CR3D
put "$LOCAL_TMP_DIR/$NEW_INI_FILENAME"
bye
EOF
else
    # Key-based authentication (no password needed)
    sftp -oPort=$SFTP_PORT "$SFTP_USER@$SFTP_HOST" <<EOF
cd $SFTP_BASE_DIR
put "$LOCAL_TMP_DIR/vendor_indices.zip"
cd CR3D
put "$LOCAL_TMP_DIR/$NEW_INI_FILENAME"
bye
EOF
fi

# Cleanup
rm -rf "$LOCAL_TMP_DIR"

echo "SFTP update completed successfully!"
