#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="robomimic-launcher.service"
SOURCE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_FILE="${SOURCE_DIR}/${SERVICE_NAME}"
TARGET_FILE="/etc/systemd/system/${SERVICE_NAME}"

if [[ ! -f "${SOURCE_FILE}" ]]; then
  echo "missing service file: ${SOURCE_FILE}" >&2
  exit 1
fi

echo "Installing ${SERVICE_NAME} to ${TARGET_FILE}"
sudo install -m 0644 "${SOURCE_FILE}" "${TARGET_FILE}"
sudo systemctl daemon-reload
sudo systemctl enable "${SERVICE_NAME}"
sudo systemctl restart "${SERVICE_NAME}"
sudo systemctl status "${SERVICE_NAME}" --no-pager
