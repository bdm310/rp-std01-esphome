#!/usr/bin/env bash
# Uploads a firmware image over Wi-Fi through Tasmota's safeboot partition, the same way the
# Tasmota web page does. The safeboot partition itself is never changed.
set -euo pipefail

if [ $# -ne 2 ]; then
  echo "usage: upload.sh <device-ip> <image>" >&2
  echo "Uploads <image> (an OTA image, for example ESPHome's firmware.ota.bin) to the Tasmota unit at <device-ip>." >&2
  exit 2
fi
ip=$1
image=$2
if [ ! -f "$image" ]; then
  echo "Error: $image is not a file." >&2
  exit 2
fi
size=$(wc -c < "$image" | tr -d ' ')
case "$(basename "$image")" in
  *[Ff][Aa][Cc][Tt][Oo][Rr][Yy]*)
    echo "Error: $image is a factory image, for a serial flash from address 0. Upload the OTA image (firmware.ota.bin) instead; nothing was uploaded." >&2
    exit 2
    ;;
esac
timeout_s=120

# The first request asks the running firmware to restart into safeboot and answers "false".
# Safeboot answers "true" once it is ready to receive the image. While the unit restarts, nothing
# answers at all.
echo "Restarting $ip into safeboot..."
deadline=$((SECONDS + timeout_s))
while :; do
  reply=$(curl -s -m 5 -w '\n%{http_code}' "http://$ip/u4?u4=fct&api=" || true)
  status=${reply##*$'\n'}
  body=${reply%$'\n'*}
  case "$status:$body" in
    200:true)
      break
      ;;
    200:false | 000:*)
      ;;
    401:*)
      echo "Error: $ip asks for Tasmota's web password. Clear it in Tasmota (WebPassword 0), then try again; nothing was uploaded." >&2
      exit 1
      ;;
    *)
      echo "Error: $ip answered HTTP $status, not as Tasmota does: is it running Tasmota? Nothing was uploaded." >&2
      exit 1
      ;;
  esac
  if [ "$SECONDS" -ge "$deadline" ]; then
    echo "Error: $ip did not come back as Tasmota's safeboot within $timeout_s s; nothing was uploaded." >&2
    exit 1
  fi
  sleep 1
done

echo "Uploading $image ($size bytes)..."
# The image goes through stdin, so curl does not read ';' or ',' in its path as form options.
if ! reply=$(curl -sS -f -m 300 -F "u2=@-;filename=firmware.bin" "http://$ip/u2?fsz=$size" < "$image"); then
  echo "Error: the upload to $ip failed (see the curl message above)." >&2
  exit 1
fi
text=$(printf '%s' "$reply" | sed 's/<[^>]*>/ /g' | tr -s ' \n')
printf '%s\n' "$text" | tail -c 300
echo
if ! printf '%s' "$text" | grep -qi "successful"; then
  echo "Error: Tasmota did not report a successful upload." >&2
  exit 1
fi
echo "Done. The device restarts into the new firmware."
