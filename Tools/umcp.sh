#!/usr/bin/env bash
# Talk to the Unreal editor's MCP server over streamable HTTP.
#   umcp.sh <method> <json-params>
# Keeps a session id in a temp file so calls share one MCP session.
URL=http://127.0.0.1:8000/mcp
SIDFILE=/tmp/.umcp_session

hdrs=(-H "Content-Type: application/json" -H "Accept: application/json, text/event-stream")

new_session() {
  curl -s -m 10 -D - -o /dev/null -X POST "$URL" "${hdrs[@]}" \
    -d '{"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2024-11-05","capabilities":{},"clientInfo":{"name":"claude","version":"1"}}}' \
    | tr -d '\r' | awk -F': ' 'tolower($1)=="mcp-session-id"{print $2}' > "$SIDFILE"
  SID=$(cat "$SIDFILE")
  curl -s -m 10 -X POST "$URL" "${hdrs[@]}" -H "Mcp-Session-Id: $SID" \
    -d '{"jsonrpc":"2.0","method":"notifications/initialized"}' >/dev/null
}

[ -s "$SIDFILE" ] || new_session
SID=$(cat "$SIDFILE")

curl -s -m 60 -X POST "$URL" "${hdrs[@]}" -H "Mcp-Session-Id: $SID" \
  -d "{\"jsonrpc\":\"2.0\",\"id\":9,\"method\":\"$1\",\"params\":$2}"
