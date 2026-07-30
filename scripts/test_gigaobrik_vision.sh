#!/usr/bin/env bash
set -eu

overrides_dir="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
export GZ_SIM_RESOURCE_PATH="${overrides_dir}/models${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
export GZ_PARTITION="gigaobrik_vision_test_${RANDOM}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"

log_file="$(mktemp)"
sim_pid=""

cleanup() {
  if [ -n "${sim_pid}" ]; then
    kill "${sim_pid}" 2>/dev/null || true
    wait "${sim_pid}" 2>/dev/null || true
  fi
  rm -f "${log_file}"
}
trap cleanup EXIT

gz sim -s -r "${overrides_dir}/worlds/gigaobrik_vision_test.sdf" \
  >"${log_file}" 2>&1 &
sim_pid=$!

topics=""
for _ in $(seq 1 30); do
  sleep 2
  topics="$(gz topic -l 2>/dev/null || true)"
  if grep -q 'orange_pi_camera_bottom/link/orange_pi_camera_link/sensor/imager/image$' \
      <<<"${topics}"; then
    break
  fi
done

bottom_topic="$(grep 'orange_pi_camera_bottom/link/orange_pi_camera_link/sensor/imager/image$' \
  <<<"${topics}" | head -n 1)"
front_topic="$(grep 'orange_pi_camera_front/link/orange_pi_camera_link/sensor/imager/image$' \
  <<<"${topics}" | head -n 1)"
bottom_info_topic="$(grep 'orange_pi_camera_bottom/link/orange_pi_camera_link/sensor/imager/camera_info$' \
  <<<"${topics}" | head -n 1)"
front_info_topic="$(grep 'orange_pi_camera_front/link/orange_pi_camera_link/sensor/imager/camera_info$' \
  <<<"${topics}" | head -n 1)"

if [ -z "${bottom_topic}" ] || [ -z "${front_topic}" ] \
    || [ -z "${bottom_info_topic}" ] || [ -z "${front_info_topic}" ]; then
  echo "Expected camera topics were not created."
  tail -n 80 "${log_file}"
  exit 1
fi

printf '%s\n' "${bottom_topic}" "${bottom_info_topic}" \
  "${front_topic}" "${front_info_topic}"

if ! timeout 45 gz topic -e -t "${bottom_topic}" \
    | grep -m 1 -q '^width: 1408$'; then
  echo "Bottom camera did not emit a 1408-pixel-wide operational frame."
  tail -n 80 "${log_file}"
  exit 1
fi

if ! timeout 45 gz topic -e -t "${front_topic}" \
    | grep -m 1 -q '^width: 1408$'; then
  echo "Front camera did not emit a 1408-pixel-wide operational frame."
  tail -n 80 "${log_file}"
  exit 1
fi

echo "Both Orange Pi cameras emitted 1408x792 OV13850 operational frames."
