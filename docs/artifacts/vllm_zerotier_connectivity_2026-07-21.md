# vLLM and ZeroTier connectivity review

Date: 21 July 2026

## Verdict

The vLLM host is not reachable because the local ZeroTier daemon is offline at the overlay control-plane level. The daemon process is running and the virtual interfaces/routes remain configured, but the authoritative local API reports no active peer paths.

This is a networking/ZeroTier availability failure before the vLLM HTTP service. It is not currently possible to distinguish a stopped vLLM process from a remote firewall problem because the remote member cannot be reached.

## Evidence

### Local ZeroTier daemon

- Service: `zerotier-one.service`, active and enabled.
- Version: `1.16.2`.
- Node address: `211ac984ac`.
- Local API status: `online: false`.
- TCP fallback: `tcpFallbackActive: true`.
- Peer state: root and leaf peers report `latency: -1` and `paths: []`.
- Network status: both joined networks report `status: OK`, but that only confirms local interface configuration.

The daemon API was queried read-only through the host network. The authentication token was not printed or persisted.

### Local overlay networks

| Network | Name | Local address | Interface | Managed route |
| --- | --- | --- | --- | --- |
| `3b19b3a716937e29` | `WatsonOW` | `10.88.140.44/24` | `zttqhsbwj7` | `10.88.140.0/24` |
| `cf719fd540557d9d` | `my-first-network` | `10.7.138.44/24` | `ztdiy4vkg5` | `10.7.138.0/24` |

Both interfaces are `UP` and both routes select the expected ZeroTier device. This is why a route lookup alone appears healthy.

### Target reachability

| Target | Route | Neighbor | Result |
| --- | --- | --- | --- |
| `10.7.138.215:8004` | `ztdiy4vkg5`, source `10.7.138.44` | `FAILED` | HTTP status 000, no route/connection |
| `10.88.140.94:4000` | `zttqhsbwj7`, source `10.88.140.44` | `FAILED` | timeout, no HTTP response |
| `10.88.140.94:18080` | `zttqhsbwj7`, source `10.88.140.44` | `FAILED` | no route/connection |

The same failures occur from the `nao_ros2` container, which uses host networking. Docker is therefore not the differentiating cause.

### Documented endpoint drift

The long-running `/home/juanbeck/zerotier-llm-proxy` repository documents the canonical ZeroTier proxy as:

- LiteLLM: `http://10.88.140.94:4000/v1`
- DFlash compatibility proxy: `http://10.88.140.94:18080/v1`

The NAO stack currently targets `http://10.7.138.215:8004`. That is a separate ZeroTier network/address and is not the documented LiteLLM endpoint. Both paths are currently unreachable, but the address mismatch must be resolved after ZeroTier comes online.

## Likely failure classification

1. Primary: local ZeroTier control-plane connectivity is offline. The peer table has no paths, so the overlay cannot deliver packets to either remote address.
2. Secondary: the configured `10.7.138.215:8004` endpoint has drifted from the documented `10.88.140.94:4000` or `:18080` proxy endpoints, or it belongs to a separate remote deployment that is currently offline.
3. Not evidenced: vLLM process health, port binding, model inventory, or Windows firewall behavior. Those require a working ZeroTier path or a check on the remote host.

## Safe recovery sequence

Run locally with a terminal that can authenticate for `sudo`:

```bash
sudo systemctl restart zerotier-one
sudo zerotier-cli info
sudo zerotier-cli listnetworks
sudo zerotier-cli listpeers
ip route get 10.7.138.215
ip route get 10.88.140.94
```

Accept the recovery only when `zerotier-cli info` reports `ONLINE` and the relevant peers have non-empty paths and non-negative latency. Do not leave or rejoin either network unless `listnetworks` reports `ACCESS_DENIED` or the membership is absent. Both networks currently have `allowManaged=1` and the expected local addresses.

Then probe both documented and stack-configured endpoints:

```bash
curl --connect-timeout 5 --max-time 15 http://10.7.138.215:8004/v1/models
curl --connect-timeout 5 --max-time 15 http://10.88.140.94:4000/v1/models
curl --connect-timeout 5 --max-time 15 http://10.88.140.94:18080/v1/models
```

On the Windows serving host, verify the actual ZeroTier IPv4 address and listener before changing the ROS launch configuration:

```powershell
zerotier-cli info
zerotier-cli listnetworks
Get-NetIPAddress -AddressFamily IPv4 | Where-Object InterfaceAlias -Match 'ZeroTier'
Get-NetTCPConnection -State Listen -LocalPort 4000,8004,18080
Invoke-WebRequest http://127.0.0.1:8004/v1/models
Invoke-WebRequest http://127.0.0.1:4000/v1/models
```

If the service is healthy locally but not remotely reachable, check Windows Firewall for the active ZeroTier network profile and the actual listening port. The vLLM/LiteLLM server must bind to the intended interface, not only `127.0.0.1`.

## Current stack impact

The ROS stack remains operational on the Ollama fallback. Its chatbot and planner are configured for `gemma4:cloud`, while the vLLM path is not scoreable until ZeroTier returns online and a named `/v1/models` probe succeeds. No stack restart or ZeroTier mutation was performed during this review.
