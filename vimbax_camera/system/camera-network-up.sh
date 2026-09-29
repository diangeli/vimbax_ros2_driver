#!/bin/sh
# At boot nvethernet races us: eth0 can come back up between our link-down and the MTU change
# ("must be stopped to change its MTU"), or the MTU is reset to 1500 right after it is set.
IF=eth0

ip addr replace 192.168.1.140/24 dev "$IF"
# The cameras send PTP via their gateway 192.168.1.1; answering as it lets PTP lock.
ip addr replace 192.168.1.1/24 dev "$IF"

for attempt in $(seq 1 15); do
    ip link set "$IF" down
    sleep 1
    ip link set "$IF" mtu 9000 || echo "attempt $attempt: MTU change refused"
    ip link set "$IF" up
    sleep 2
    mtu=$(cat "/sys/class/net/$IF/mtu")
    if [ "$mtu" -ge 8000 ]; then
        echo "$IF MTU $mtu after $attempt attempt(s)"
        exit 0
    fi
done

echo "$IF MTU stuck at $(cat "/sys/class/net/$IF/mtu"): jumbo frames unavailable, the camera loader will refuse to start" >&2
exit 0
