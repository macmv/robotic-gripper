set -eou pipefail

cd -- "$(dirname -- "$0")"

if ! command -v arduino-cli &> /dev/null; then
  echo "Please install arduino-cli"
  exit 1
fi

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <port>"
  echo ""
  echo "Ports found:"
  arduino-cli board list
  exit 1
fi

PORT=$1

arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328old
arduino-cli upload --fqbn arduino:avr:nano:cpu=atmega328old --port $PORT
