#!/usr/bin/env bash
set -euo pipefail

: "${ACCEPT_EULA:=Y}"
: "${PRIVACY_CONSENT:=Y}"
export ACCEPT_EULA PRIVACY_CONSENT
exec "$@"
