import base64
import hashlib
import json
import re
import socket
import struct
import threading
import time
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Dict, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from hiking_lora_sim.scenario import (
    BASE_STATION,
    LORA_NODES,
    RADIO_OBSTACLES,
    TRAILS,
    WORLD_EXTENT,
    load_scenario_yaml,
)


_WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
_STATUS_RE = re.compile(
    r"(?P<hiker_id>\S+)\s+track=(?P<trail>\S+)\s+"
    r"x=(?P<x>[+-]?[\d.]+)\s+y=(?P<y>[+-]?[\d.]+)\s+"
    r"terrain_z=(?P<terrain_z>[+-]?[\d.]+)\s+alt=(?P<altitude>[\d.]+)m\s+"
    r"gps=\((?P<gps_lat>[+-]?[\d.]+),(?P<gps_lon>[+-]?[\d.]+)\)\s+"
    r"bat=(?P<battery>[\d.]+)%\s+"
    r"dop=(?P<dop>[\d.]+)\s+gps_err=(?P<gps_err>[\d.]+)m\s+"
    r".*?weather=(?P<weather>\S+)"
)
_CONTROL_RE = re.compile(r"\bcontrol=(?P<control>\S+)")
_SOS_RE = re.compile(r"\bsos=(?P<sos>true|false|1|0|yes|no)", re.IGNORECASE)
_SOS_COUNT_RE = re.compile(r"\bsos_count=(?P<count>\d+)")


_DASHBOARD_HTML = r"""<!doctype html>
<html lang="id">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>SiLacak React Dashboard</title>
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
  <style>
    :root {
      --bg: #eef2f7;
      --panel: #ffffff;
      --line: #d5dce8;
      --text: #101827;
      --muted: #64748b;
      --green: #15803d;
      --red: #dc2626;
      --amber: #b45309;
      --blue: #2563eb;
      --violet: #7c3aed;
      --teal: #0f766e;
    }

    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: var(--bg);
      color: var(--text);
      overflow: hidden;
    }

    .app {
      height: 100vh;
    }

    .layout {
      position: relative;
      height: 100vh;
      min-height: 100vh;
      overflow: hidden;
    }

    #map {
      position: absolute;
      inset: 0;
      z-index: 0;
      height: 100vh;
      min-height: 100vh;
      background:
        linear-gradient(0deg, rgba(15, 23, 42, 0.035) 1px, transparent 1px),
        linear-gradient(90deg, rgba(15, 23, 42, 0.035) 1px, transparent 1px),
        #edf5f8;
      background-size: 24px 24px;
    }

    .sidebar {
      position: absolute;
      top: 8px;
      right: 8px;
      bottom: 8px;
      z-index: 500;
      width: min(390px, calc(100vw - 24px));
      min-height: 0;
      max-height: calc(100vh - 16px);
      overflow: auto;
      display: flex;
      flex-direction: column;
      gap: 8px;
      padding: 8px;
      background: #f8fafc;
      border-left: 1px solid var(--line);
    }

    .panel {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 10px;
    }

    .panel h2 {
      margin: 0 0 10px;
      color: #334155;
      font-size: 13px;
      letter-spacing: 0.04em;
      text-transform: uppercase;
    }

    .metrics {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
    }

    .summary-panel .metrics {
      gap: 12px;
    }

    .metric {
      min-height: 62px;
      padding: 9px;
      border: 1px solid #e2e8f0;
      border-radius: 6px;
      background: #fbfdff;
    }

    .summary-panel .metric {
      min-height: 80px;
      padding: 13px;
    }

    .metric strong {
      display: block;
      font-size: 21px;
      line-height: 1.1;
    }

    .summary-panel .metric strong {
      font-size: 26px;
    }

    .metric span {
      display: block;
      margin-top: 5px;
      font-size: 12px;
      color: var(--muted);
    }

    .legend {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
      font-size: 12px;
      color: #475569;
    }

    .legend-item {
      display: flex;
      align-items: center;
      gap: 7px;
      min-width: 0;
    }

    .swatch {
      flex: 0 0 auto;
      width: 13px;
      height: 13px;
      border-radius: 4px;
      border: 1px solid #ffffff;
      box-shadow: 0 0 0 1px rgba(15, 23, 42, 0.18);
    }

    table {
      width: 100%;
      border-collapse: collapse;
      font-size: 12px;
    }

    th,
    td {
      padding: 7px 6px;
      border-bottom: 1px solid #e2e8f0;
      text-align: left;
      vertical-align: top;
    }

    th {
      background: #f8fafc;
      color: #475569;
      font-weight: 700;
    }

    .hiker-row {
      cursor: pointer;
      transition: background 0.15s ease, box-shadow 0.15s ease;
    }

    .hiker-row:hover {
      background: #f1f5f9;
    }

    .hiker-row.selected td {
      background: #dbeafe;
      border-top: 1px solid #93c5fd;
      border-bottom: 1px solid #93c5fd;
    }

    .hiker-row.selected td:first-child {
      box-shadow: inset 4px 0 0 var(--blue);
    }

    .hiker-row.selected strong {
      color: #1d4ed8;
    }

    .small { color: var(--muted); font-size: 12px; }
    .mono { font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; }
    .ok-text { color: var(--green); font-weight: 700; }
    .drop-text { color: var(--amber); font-weight: 700; }
    .sos-text { color: var(--red); font-weight: 800; }

    .event-list,
    .sos-list,
    .link-list {
      display: flex;
      flex-direction: column;
      gap: 8px;
      max-height: 230px;
      overflow: auto;
    }

    .event,
    .sos-card,
    .link-card {
      padding: 8px;
      border: 1px solid #e2e8f0;
      border-radius: 6px;
      background: #fbfdff;
      font-size: 12px;
    }

    .event.ok { border-left: 4px solid var(--green); }
    .event.drop { border-left: 4px solid var(--amber); }
    .sos-card {
      border-left: 4px solid var(--red);
      background: #fff7f7;
    }
    .link-card { border-left: 4px solid var(--blue); }

    .hiker-marker,
    .relay-marker,
    .base-marker {
      display: grid;
      place-items: center;
      border: 3px solid white;
      box-shadow: 0 2px 12px rgba(15, 23, 42, 0.35);
    }

    .hiker-marker {
      width: 30px;
      height: 30px;
      border-radius: 50%;
      background: var(--blue);
      color: white;
      font-size: 10px;
      font-weight: 800;
    }

    .hiker-marker.selected {
      width: 38px;
      height: 38px;
      background: #0f172a;
      border-color: #facc15;
      font-size: 12px;
    }

    .hiker-marker.sos {
      background: var(--red);
      animation: pulse 0.9s infinite;
    }

    .relay-marker {
      width: 22px;
      height: 22px;
      border-radius: 6px;
      background: var(--violet);
    }

    .base-marker {
      width: 24px;
      height: 24px;
      border-radius: 50%;
      background: var(--teal);
    }

    @keyframes pulse {
      0% { transform: scale(1); box-shadow: 0 0 0 0 rgba(220, 38, 38, 0.55); }
      70% { transform: scale(1.12); box-shadow: 0 0 0 12px rgba(220, 38, 38, 0); }
      100% { transform: scale(1); box-shadow: 0 0 0 0 rgba(220, 38, 38, 0); }
    }

    @media (max-width: 960px) {
      .layout {
        min-height: auto;
        overflow: visible;
      }
      #map {
        position: relative;
        min-height: 58vh;
      }
      body { overflow: auto; }
      .sidebar {
        position: relative;
        inset: auto;
        width: auto;
        max-height: none;
        border-left: 0;
        border-top: 1px solid var(--line);
      }
    }
  </style>
</head>
<body>
  <div id="root"></div>

  <script src="https://unpkg.com/react@18/umd/react.production.min.js"></script>
  <script src="https://unpkg.com/react-dom@18/umd/react-dom.production.min.js"></script>
  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
  <script>
    const e = React.createElement;
    const { useEffect, useMemo, useRef, useState } = React;

    const routeColors = {
      ridge_route: "#2563eb",
      valley_route: "#f97316",
      crater_route: "#dc2626"
    };

    function xy(x, y) {
      return [Number(y), Number(x)];
    }

    function n(value, digits = 1) {
      const parsed = Number(value);
      if (!Number.isFinite(parsed)) return "-";
      return parsed.toFixed(digits);
    }

    function labelForHiker(id) {
      const parts = String(id || "hiker").split("_");
      return parts.length > 1 ? parts[1] : "1";
    }

    function iconSizeFor(className) {
      if (className.includes("hiker-marker") && className.includes("selected")) return 38;
      if (className.includes("hiker-marker")) return 30;
      if (className.includes("base-marker")) return 24;
      if (className.includes("relay-marker")) return 22;
      return 32;
    }

    function leafletIcon(className, label = "") {
      const size = iconSizeFor(className);
      return L.divIcon({
        className: "",
        html: `<div class="${className}">${label}</div>`,
        iconSize: [size, size],
        iconAnchor: [size / 2, size / 2]
      });
    }

    function stationLookup(scenario, hikers, event) {
      const lookup = {};
      if (scenario && scenario.base_station) lookup[scenario.base_station.name] = scenario.base_station;
      (scenario && scenario.lora_nodes || []).forEach(node => { lookup[node.name] = node; });
      Object.values(hikers || {}).forEach(hiker => {
        if (hiker && hiker.hiker_id && hiker.x !== undefined && hiker.y !== undefined) {
          lookup[hiker.hiker_id] = hiker;
        }
      });
      if (lookup.hiker && !lookup.hiker_1) lookup.hiker_1 = lookup.hiker;
      if (lookup.hiker_1 && !lookup.hiker) lookup.hiker = lookup.hiker_1;

      if (
        event &&
        event.hiker_id &&
        event.hiker_local &&
        (!lookup[event.hiker_id] || lookup[event.hiker_id].x === undefined || lookup[event.hiker_id].y === undefined)
      ) {
        lookup[event.hiker_id] = {
          hiker_id: event.hiker_id,
          x: event.hiker_local.x,
          y: event.hiker_local.y
        };
      }
      if (lookup.hiker && !lookup.hiker_1) lookup.hiker_1 = lookup.hiker;
      if (lookup.hiker_1 && !lookup.hiker) lookup.hiker = lookup.hiker_1;
      return lookup;
    }

    function linkColor(link) {
      const margin = Number(link.margin_db);
      if (!Number.isFinite(margin)) return "#2563eb";
      if (margin >= 10) return "#15803d";
      if (margin >= 0) return "#b45309";
      return "#dc2626";
    }

    function distanceBetween(a, b) {
      const dx = Number(a.x) - Number(b.x);
      const dy = Number(a.y) - Number(b.y);
      if (!Number.isFinite(dx) || !Number.isFinite(dy)) return Infinity;
      return Math.sqrt(dx * dx + dy * dy);
    }

    function estimatedNearestRelayEvent(scenario, hikers, selectedHikerId) {
      const hiker = hikers && hikers[selectedHikerId];
      const nodes = scenario && Array.isArray(scenario.lora_nodes) ? scenario.lora_nodes : [];
      if (!hiker || hiker.x === undefined || hiker.y === undefined || !nodes.length) return null;
      let bestNode = null;
      let bestDistance = Infinity;
      nodes.forEach(node => {
        const distance = distanceBetween(hiker, node);
        if (distance < bestDistance) {
          bestDistance = distance;
          bestNode = node;
        }
      });
      if (!bestNode) return null;
      return {
        hiker_id: selectedHikerId,
        delivered: true,
        estimated: true,
        links: [{
          from: selectedHikerId,
          to: bestNode.name,
          distance_m: bestDistance,
          rx_dbm: null,
          snr_db: null,
          margin_db: 12,
          estimated: true
        }]
      };
    }

    function latestLinkEvent(events, selectedHikerId, scenario, hikers) {
      if (!selectedHikerId) return null;
      const actual = (events || []).find(ev =>
        Array.isArray(ev.links) &&
        ev.links.length > 0 &&
        ev.hiker_id === selectedHikerId
      );
      return actual || estimatedNearestRelayEvent(scenario, hikers, selectedHikerId);
    }

    function MapView({ scenario, hikers, events, selectedHikerId }) {
      const mapRef = useRef(null);
      const mapElRef = useRef(null);
      const scenarioLayerRef = useRef(null);
      const hikerLayerRef = useRef(null);
      const loraLayerRef = useRef(null);

      const latestEventWithLinks = useMemo(() => {
        return latestLinkEvent(events, selectedHikerId, scenario, hikers);
      }, [events, selectedHikerId, scenario, hikers]);

      useEffect(() => {
        if (!window.L || mapRef.current) return;
        const map = L.map(mapElRef.current, {
          crs: L.CRS.Simple,
          minZoom: -3,
          maxZoom: 4,
          zoomControl: true
        });
        mapRef.current = map;
        scenarioLayerRef.current = L.layerGroup().addTo(map);
        hikerLayerRef.current = L.layerGroup().addTo(map);
        loraLayerRef.current = L.layerGroup().addTo(map);
        map.setView([0, 0], 0);
        return () => map.remove();
      }, []);

      useEffect(() => {
        const map = mapRef.current;
        const layer = scenarioLayerRef.current;
        if (!map || !layer || !scenario) return;
        layer.clearLayers();

        const extent = Number(scenario.extent || 120);
        const bounds = [[-extent, -extent], [extent, extent]];
        L.rectangle(bounds, {
          color: "#94a3b8",
          weight: 1,
          fillOpacity: 0.06
        }).addTo(layer);

        Object.entries(scenario.trails || {}).forEach(([name, points]) => {
          const line = L.polyline(points.map(point => xy(point[0], point[1])), {
            color: routeColors[name] || "#334155",
            weight: 3,
            opacity: 0.82
          }).addTo(layer);
          line.bindPopup(`<strong>${name}</strong><br>Jalur pendakian`);
        });

        (scenario.obstacles || []).forEach(obs => {
          const circle = L.circle(xy(obs.x, obs.y), {
            radius: Number(obs.radius),
            color: "#b45309",
            weight: 1,
            fillColor: "#fbbf24",
            fillOpacity: 0.13
          }).addTo(layer);
          circle.bindPopup(`<strong>${obs.name}</strong><br>${obs.kind}<br>loss ${obs.loss_db} dB`);
        });

        (scenario.lora_nodes || []).forEach(node => {
          const marker = L.marker(xy(node.x, node.y), {
            icon: leafletIcon("relay-marker")
          }).addTo(layer);
          marker.bindPopup(`<strong>${node.name}</strong><br>LoRa relay node`);
        });

        if (scenario.base_station) {
          const base = scenario.base_station;
          const marker = L.marker(xy(base.x, base.y), {
            icon: leafletIcon("base-marker")
          }).addTo(layer);
          marker.bindPopup(`<strong>${base.name}</strong><br>Base station`);
        }

        map.fitBounds(bounds, {
          paddingTopLeft: [8, 8],
          paddingBottomRight: [408, 8]
        });
        map.setZoom(Math.min(map.getMaxZoom(), map.getZoom() + 2));
      }, [scenario]);

      useEffect(() => {
        const layer = hikerLayerRef.current;
        if (!layer) return;
        layer.clearLayers();
        Object.values(hikers || {}).forEach(hiker => {
          if (hiker.x === undefined || hiker.y === undefined) return;
          const id = hiker.hiker_id || "hiker";
          const isSelected = selectedHikerId === id;
          const marker = L.marker(xy(hiker.x, hiker.y), {
            icon: leafletIcon(`hiker-marker${isSelected ? " selected" : ""}${hiker.sos_active ? " sos" : ""}`, labelForHiker(id)),
            zIndexOffset: isSelected ? 1000 : 0
          }).addTo(layer);
          const gps = hiker.gps || {};
          marker.bindPopup(`
            <strong>${id}</strong><br>
            x=${n(hiker.x, 2)}, y=${n(hiker.y, 2)}<br>
            GPS ${gps.lat ?? "-"}, ${gps.lon ?? "-"}<br>
            Battery ${hiker.battery_pct ?? "-"}%<br>
            ${hiker.sos_active ? "<strong>SOS ACTIVE</strong>" : "Normal"}
          `);
        });
      }, [hikers, selectedHikerId]);

      useEffect(() => {
        const layer = loraLayerRef.current;
        if (!layer) return;
        layer.clearLayers();
        if (!latestEventWithLinks) return;
        const lookup = stationLookup(scenario, hikers, latestEventWithLinks);
        latestEventWithLinks.links.forEach(link => {
          const from = lookup[link.from];
          const to = lookup[link.to];
          if (!from || !to || from.x === undefined || to.x === undefined) return;
          const line = L.polyline([xy(from.x, from.y), xy(to.x, to.y)], {
            color: linkColor(link),
            weight: 3,
            opacity: link.estimated ? 0.55 : 0.78,
            dashArray: link.estimated ? "5 8" : (latestEventWithLinks.delivered ? null : "7 7")
          }).addTo(layer);
          line.bindPopup(`
            <strong>${link.from} -> ${link.to}</strong><br>
            ${link.estimated ? "perkiraan relay terdekat<br>" : ""}
            distance ${n(link.distance_m, 0)} m<br>
            RSSI ${n(link.rx_dbm, 1)} dBm<br>
            SNR ${n(link.snr_db, 1)} dB<br>
            margin ${n(link.margin_db, 1)} dB
          `);
        });
      }, [scenario, hikers, latestEventWithLinks]);

      return e("div", { id: "map", ref: mapElRef });
    }

    function useDashboardSocket() {
      const [connection, setConnection] = useState({ label: "Connecting...", level: "warn" });
      const [scenario, setScenario] = useState(null);
      const [hikers, setHikers] = useState({});
      const [stats, setStats] = useState({ total_packets: 0, delivered_packets: 0, pdr_pct: 0 });
      const [events, setEvents] = useState([]);
      const [sosEvents, setSosEvents] = useState([]);

      useEffect(() => {
        let closed = false;
        let ws = null;

        function connect() {
          if (closed) return;
          const scheme = location.protocol === "https:" ? "wss" : "ws";
          ws = new WebSocket(`${scheme}://${location.host}/ws`);
          ws.onopen = () => setConnection({ label: "WebSocket connected", level: "ok" });
          ws.onerror = () => setConnection({ label: "WebSocket error", level: "warn" });
          ws.onclose = () => {
            setConnection({ label: "Disconnected - reconnecting", level: "bad" });
            if (!closed) setTimeout(connect, 1200);
          };
          ws.onmessage = message => {
            const data = JSON.parse(message.data);
            if (data.type === "snapshot") {
              setScenario(data.scenario);
              setHikers(data.hikers || {});
              setStats(data.stats || {});
              setEvents(data.events || []);
              setSosEvents(data.sos_events || []);
              return;
            }
            if (data.type === "status") {
              setHikers(prev => ({
                ...prev,
                [data.hiker_id]: { ...(prev[data.hiker_id] || {}), ...data }
              }));
              return;
            }
            if (data.type === "network_event") {
              setEvents(prev => [data.event, ...prev].slice(0, 60));
              if (data.stats) setStats(data.stats);
              const ev = data.event || {};
              if (ev.hiker_id && ev.hiker_local) {
                setHikers(prev => ({
                  ...prev,
                  [ev.hiker_id]: {
                    ...(prev[ev.hiker_id] || {}),
                    hiker_id: ev.hiker_id,
                    x: ev.hiker_local.x,
                    y: ev.hiker_local.y,
                    gps: ev.gps,
                    sos_active: Boolean(ev.sos_active || (prev[ev.hiker_id] || {}).sos_active)
                  }
                }));
              }
              return;
            }
            if (data.type === "sos") {
              setSosEvents(prev => [data, ...prev].slice(0, 30));
              setHikers(prev => ({
                ...prev,
                [data.hiker_id]: {
                  ...(prev[data.hiker_id] || {}),
                  hiker_id: data.hiker_id,
                  sos_active: data.active !== false
                }
              }));
            }
          };
        }

        connect();
        return () => {
          closed = true;
          if (ws) ws.close();
        };
      }, []);

      return { connection, scenario, hikers, stats, events, sosEvents };
    }

    function Metrics({ hikers, stats, events }) {
      const hikerList = Object.values(hikers || {});
      const sosCount = hikerList.filter(h => h.sos_active).length;
      const latest = events && events[0];
      return e("div", { className: "metrics" },
        e("div", { className: "metric" }, e("strong", null, hikerList.length), e("span", null, "Pendaki aktif")),
        e("div", { className: "metric" }, e("strong", null, `${n(stats.pdr_pct || 0, 1)}%`), e("span", null, "PDR sesi")),
        e("div", { className: "metric" }, e("strong", null, stats.total_packets || 0), e("span", null, "Paket LoRa")),
        e("div", { className: "metric" }, e("strong", null, sosCount), e("span", null, "SOS aktif")),
        e("div", { className: "metric" }, e("strong", null, latest ? n(latest.snr_db, 1) : "-"), e("span", null, "SNR terbaru dB")),
        e("div", { className: "metric" }, e("strong", null, latest ? (latest.entry_node || "-") : "-"), e("span", null, "Entry relay"))
      );
    }

    function Legend() {
      const items = [
        ["#2563eb", "Hiker / rute ridge"],
        ["#f97316", "Rute valley"],
        ["#dc2626", "Rute crater"],
        ["#7c3aed", "LoRa relay"],
        ["#0f766e", "Base station"],
        ["#fbbf24", "Obstacle radio"],
        ["#15803d", "Link LoRa kuat"],
        ["#dc2626", "SOS / link buruk"]
      ];
      return e("div", { className: "legend" },
        items.map(([color, label]) =>
          e("div", { className: "legend-item", key: label },
            e("span", { className: "swatch", style: { background: color } }),
            e("span", null, label)
          )
        )
      );
    }

    function HikerTable({ hikers, selectedHikerId, onSelectHiker }) {
      const rows = Object.values(hikers || {}).sort((a, b) => String(a.hiker_id).localeCompare(String(b.hiker_id)));
      return e("table", null,
        e("thead", null, e("tr", null, e("th", null, "ID"), e("th", null, "Posisi"), e("th", null, "Status"))),
        e("tbody", null,
          rows.length ? rows.map(hiker =>
            e("tr", {
              key: hiker.hiker_id,
              className: `hiker-row${selectedHikerId === hiker.hiker_id ? " selected" : ""}`,
              onClick: () => onSelectHiker(hiker.hiker_id)
            },
              e("td", null,
                e("strong", null, hiker.hiker_id),
                e("br"),
                e("span", { className: "small" }, hiker.trail || "-")
              ),
              e("td", null,
                e("span", { className: "mono" }, hiker.x === undefined ? "-" : `${n(hiker.x, 1)}, ${n(hiker.y, 1)}`),
                e("br"),
                e("span", { className: "small" }, `${hiker.battery_pct ?? "-"}% battery`)
              ),
              e("td", null,
                e("span", { className: hiker.sos_active ? "sos-text" : "ok-text" }, hiker.sos_active ? "SOS" : "Normal"),
                hiker.manual_control ? e("span", null,
                  e("br"),
                  e("span", { className: "small" }, "manual")
                ) : null
              )
            )
          ) : e("tr", null, e("td", { colSpan: 3, className: "small" }, "Menunggu status hiker."))
        )
      );
    }

    function EventList({ events }) {
      const rows = (events || []).slice(0, 12);
      if (!rows.length) return e("div", { className: "small" }, "Menunggu event LoRa.");
      return e("div", { className: "event-list" },
        rows.map((ev, index) => {
          const route = Array.isArray(ev.route) && ev.route.length ? ev.route.join(" -> ") : "-";
          return e("div", { className: `event ${ev.delivered ? "ok" : "drop"}`, key: `${ev.stamp}-${index}` },
            e("strong", null, ev.hiker_id || "hiker"),
            " ",
            e("span", { className: ev.delivered ? "ok-text" : "drop-text" }, ev.delivered ? "delivered" : (ev.drop_reason || "drop")),
            e("br"),
            e("span", { className: "small" }, `route: ${route}`),
            e("br"),
            e("span", { className: "small" }, `latency ${n(ev.end_to_end_latency_ms, 0)} ms, SNR ${n(ev.snr_db, 1)} dB`)
          );
        })
      );
    }

    function LinkList({ events, selectedHikerId, scenario, hikers }) {
      const event = latestLinkEvent(events, selectedHikerId, scenario, hikers);
      if (!event) {
        return e("div", { className: "small" },
          selectedHikerId ? `Belum ada link LoRa terbaru untuk ${selectedHikerId}.` : "Klik salah satu hiker di tabel untuk melihat link LoRa."
        );
      }
      return e("div", { className: "link-list" },
        selectedHikerId ? e("div", { className: "small" }, `Dipilih: ${selectedHikerId}${event.estimated ? " - perkiraan relay terdekat" : ""}`) : null,
        event.links.map((link, index) =>
          e("div", { className: "link-card", key: `${link.from}-${link.to}-${index}` },
            e("strong", null, `${link.from} -> ${link.to}`),
            e("br"),
            e("span", { className: "small" }, `RSSI ${n(link.rx_dbm, 1)} dBm, SNR ${n(link.snr_db, 1)} dB`),
            e("br"),
            e("span", { className: "small" }, `margin ${n(link.margin_db, 1)} dB, jarak ${n(link.distance_m, 0)} m`)
          )
        )
      );
    }

    function SosList({ sosEvents }) {
      const rows = (sosEvents || []).slice(0, 10);
      if (!rows.length) return e("div", { className: "small" }, "Belum ada SOS.");
      return e("div", { className: "sos-list" },
        rows.map((item, index) =>
          e("div", { className: "sos-card", key: `${item.hiker_id}-${item.stamp}-${index}` },
            e("strong", null, item.hiker_id || "hiker"),
            " SOS terdeteksi",
            e("br"),
            e("span", { className: "small" }, `source ${item.source || "-"}, t=${n(item.stamp, 1)}`)
          )
        )
      );
    }

    function App() {
      const { scenario, hikers, stats, events, sosEvents } = useDashboardSocket();
      const [selectedHikerId, setSelectedHikerId] = useState(null);
      return e("div", { className: "app" },
        e("main", { className: "layout" },
          e(MapView, { scenario, hikers, events, selectedHikerId }),
          e("aside", { className: "sidebar" },
            e("section", { className: "panel summary-panel" }, e("h2", null, "Ringkasan"), e(Metrics, { hikers, stats, events })),
            e("section", { className: "panel" }, e("h2", null, "Legenda Map"), e(Legend)),
            e("section", { className: "panel" }, e("h2", null, "SOS"), e(SosList, { sosEvents })),
            e("section", { className: "panel" }, e("h2", null, "Pendaki"), e(HikerTable, { hikers, selectedHikerId, onSelectHiker: setSelectedHikerId })),
            e("section", { className: "panel" }, e("h2", null, "Link LoRa Aktif"), e(LinkList, { events, selectedHikerId, scenario, hikers })),
            e("section", { className: "panel" }, e("h2", null, "Event Jaringan"), e(EventList, { events }))
          )
        )
      );
    }

    if (!window.React || !window.ReactDOM || !window.L) {
      document.getElementById("root").innerHTML =
        "<div style='padding:16px;font-family:sans-serif'>React atau Leaflet gagal dimuat. Pastikan browser punya akses internet ke CDN.</div>";
    } else {
      ReactDOM.createRoot(document.getElementById("root")).render(e(App));
    }
  </script>
</body>
</html>
"""


def _as_bool(value, default=True) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _float_or_none(value) -> Optional[float]:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _station_dict(station) -> Dict:
    return {
        "name": station.name,
        "x": station.x,
        "y": station.y,
        "z": station.z,
        "kind": station.kind,
    }


def _obstacle_dict(obstacle) -> Dict:
    return {
        "name": obstacle.name,
        "x": obstacle.x,
        "y": obstacle.y,
        "radius": obstacle.radius,
        "loss_db": obstacle.loss_db,
        "kind": obstacle.kind,
    }


class WebSocketHub:
    def __init__(self) -> None:
        self._clients = set()
        self._lock = threading.Lock()

    def register(self, sock: socket.socket) -> None:
        with self._lock:
            self._clients.add(sock)

    def unregister(self, sock: socket.socket) -> None:
        with self._lock:
            self._clients.discard(sock)
        try:
            sock.close()
        except OSError:
            pass

    def send_to(self, sock: socket.socket, payload: Dict) -> None:
        data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self._send_frame(sock, data)

    def broadcast(self, payload: Dict) -> None:
        data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        with self._lock:
            clients = list(self._clients)
        dead = []
        for sock in clients:
            try:
                self._send_frame(sock, data)
            except OSError:
                dead.append(sock)
        for sock in dead:
            self.unregister(sock)

    def listen_until_close(self, sock: socket.socket) -> None:
        try:
            while True:
                header = self._recv_exact(sock, 2)
                if not header:
                    return
                first, second = header
                opcode = first & 0x0F
                masked = bool(second & 0x80)
                length = second & 0x7F
                if length == 126:
                    length = struct.unpack("!H", self._recv_exact(sock, 2))[0]
                elif length == 127:
                    length = struct.unpack("!Q", self._recv_exact(sock, 8))[0]
                mask = self._recv_exact(sock, 4) if masked else b""
                payload = self._recv_exact(sock, length) if length else b""
                if masked and payload:
                    payload = bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload))
                if opcode == 0x8:
                    return
                if opcode == 0x9:
                    self._send_frame(sock, payload, opcode=0xA)
        except OSError:
            return
        finally:
            self.unregister(sock)

    @staticmethod
    def _recv_exact(sock: socket.socket, size: int) -> bytes:
        data = bytearray()
        while len(data) < size:
            chunk = sock.recv(size - len(data))
            if not chunk:
                return b""
            data.extend(chunk)
        return bytes(data)

    @staticmethod
    def _send_frame(sock: socket.socket, data: bytes, opcode: int = 0x1) -> None:
        header = bytearray([0x80 | opcode])
        length = len(data)
        if length <= 125:
            header.append(length)
        elif length <= 65535:
            header.append(126)
            header.extend(struct.pack("!H", length))
        else:
            header.append(127)
            header.extend(struct.pack("!Q", length))
        sock.sendall(bytes(header) + data)


class DashboardHttpServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, server_address, request_handler_class, dashboard_node):
        super().__init__(server_address, request_handler_class)
        self.dashboard_node = dashboard_node
        self.websocket_hub = dashboard_node.websocket_hub


class DashboardRequestHandler(BaseHTTPRequestHandler):
    server_version = "HikingLoraWebDashboard/0.1"
    protocol_version = "HTTP/1.1"

    def do_HEAD(self) -> None:
        if self.path not in {"/", "/index.html"}:
            self.send_error(HTTPStatus.NOT_FOUND, "Not found")
            return

        body = _DASHBOARD_HTML.encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()

    def do_GET(self) -> None:
        if self.path == "/ws":
            self._handle_ws()
            return
        if self.path not in {"/", "/index.html"}:
            self.send_error(HTTPStatus.NOT_FOUND, "Not found")
            return

        body = _DASHBOARD_HTML.encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args) -> None:
        return

    def _handle_ws(self) -> None:
        key = self.headers.get("Sec-WebSocket-Key")
        if not key:
            self.send_error(HTTPStatus.BAD_REQUEST, "Missing WebSocket key")
            return

        accept = base64.b64encode(
            hashlib.sha1((key + _WS_GUID).encode("ascii")).digest()
        ).decode("ascii")
        self.send_response(HTTPStatus.SWITCHING_PROTOCOLS)
        self.send_header("Upgrade", "websocket")
        self.send_header("Connection", "Upgrade")
        self.send_header("Sec-WebSocket-Accept", accept)
        self.end_headers()
        self.close_connection = True

        self.server.websocket_hub.register(self.request)
        self.server.dashboard_node.send_snapshot_to(self.request)
        self.server.websocket_hub.listen_until_close(self.request)


class WebDashboardNode(Node):
    def __init__(self) -> None:
        super().__init__("web_dashboard")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("routes_file", "")

        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        routes_file = str(self.get_parameter("routes_file").value).strip()

        scenario = load_scenario_yaml(routes_file) if routes_file else None
        self._trails = scenario.get("trails", TRAILS) if scenario else TRAILS
        self._lora_nodes = scenario.get("lora_nodes", LORA_NODES) if scenario else LORA_NODES
        self._base_station = scenario.get("base_station", BASE_STATION) if scenario else BASE_STATION
        self._radio_obstacles = (
            scenario.get("radio_obstacles", RADIO_OBSTACLES) if scenario else RADIO_OBSTACLES
        )

        self.websocket_hub = WebSocketHub()
        self._lock = threading.Lock()
        self._hikers: Dict[str, Dict] = {}
        self._events = deque(maxlen=50)
        self._sos_events = deque(maxlen=30)
        self._base_messages = deque(maxlen=30)
        self._total_packets = 0
        self._delivered_packets = 0

        self.create_subscription(String, "/hiker/status", self._on_status_text, 10)
        self.create_subscription(String, "/hikers/status", self._on_aggregate_status, 10)
        self.create_subscription(String, "/hiker/battery", self._on_battery, 10)
        self.create_subscription(String, "/hikers/battery", self._on_battery, 10)
        self.create_subscription(String, "/lora/network_event", self._on_network_event, 10)
        self.create_subscription(String, "/base_station/hiker_location", self._on_base_location, 10)
        self.create_subscription(String, "/hiker/sos", self._on_sos, 10)

        self._server = DashboardHttpServer((host, port), DashboardRequestHandler, self)
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()

        display_host = "localhost" if host in {"0.0.0.0", "::"} else host
        self.get_logger().info(f"Web dashboard ready: http://{display_host}:{port}")

    def _scenario_payload(self) -> Dict:
        return {
            "extent": WORLD_EXTENT,
            "trails": self._trails,
            "lora_nodes": [_station_dict(station) for station in self._lora_nodes],
            "base_station": _station_dict(self._base_station),
            "obstacles": [_obstacle_dict(obstacle) for obstacle in self._radio_obstacles],
        }

    def _stats_payload(self) -> Dict:
        pdr = (
            self._delivered_packets / self._total_packets * 100.0
            if self._total_packets
            else 0.0
        )
        return {
            "total_packets": self._total_packets,
            "delivered_packets": self._delivered_packets,
            "pdr_pct": round(pdr, 2),
        }

    def _snapshot_payload(self) -> Dict:
        with self._lock:
            return {
                "type": "snapshot",
                "stamp": time.time(),
                "scenario": self._scenario_payload(),
                "hikers": self._hikers,
                "stats": self._stats_payload(),
                "events": list(self._events),
                "sos_events": list(self._sos_events),
                "base_messages": list(self._base_messages),
            }

    def send_snapshot_to(self, sock: socket.socket) -> None:
        try:
            self.websocket_hub.send_to(sock, self._snapshot_payload())
        except OSError:
            self.websocket_hub.unregister(sock)

    def _broadcast(self, payload: Dict) -> None:
        self.websocket_hub.broadcast(payload)

    def _on_status_text(self, msg: String) -> None:
        payload = self._parse_status_text(msg.data)
        if not payload:
            return
        self._store_hiker(payload)
        self._broadcast({"type": "status", **payload})

    def _on_aggregate_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        payload = self._status_payload_from_json(data)
        if payload:
            self._store_hiker(payload)
            self._broadcast({"type": "status", **payload})
            return

        text = data.get("text")
        if isinstance(text, str):
            parsed = self._parse_status_text(text)
            if parsed:
                self._store_hiker(parsed)
                self._broadcast({"type": "status", **parsed})

    def _on_battery(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        hiker_id = str(data.get("hiker_id", "hiker"))
        payload = {
            "hiker_id": hiker_id,
            "battery_pct": data.get("percentage"),
            "voltage_v": data.get("voltage_v"),
            "low_power_mode": data.get("low_power_mode"),
            "node_active": data.get("node_active"),
        }
        self._store_hiker(payload)
        self._broadcast({"type": "status", **payload})

    def _on_network_event(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        hiker_id = str(event.get("hiker_id", "hiker"))
        local = event.get("hiker_local") or {}
        hiker_update = {"hiker_id": hiker_id}
        if isinstance(local, dict):
            if "x" in local:
                hiker_update["x"] = local.get("x")
            if "y" in local:
                hiker_update["y"] = local.get("y")
            if "alt_m" in local:
                hiker_update["altitude_m"] = local.get("alt_m")
        if event.get("gps"):
            hiker_update["gps"] = event.get("gps")
        if event.get("sos_active"):
            hiker_update["sos_active"] = True
            hiker_update["sos_count"] = event.get("sos_count")

        with self._lock:
            self._events.appendleft(event)
            self._total_packets += 1
            if event.get("delivered"):
                self._delivered_packets += 1
            self._merge_hiker_locked(hiker_update)
            stats = self._stats_payload()

        self._broadcast({"type": "network_event", "event": event, "stats": stats})

    def _on_base_location(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._base_messages.appendleft(payload)
        self._broadcast({"type": "base_location", "payload": payload})

    def _on_sos(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        hiker_id = str(
            data.get("target_hiker_id", data.get("hiker_id", data.get("target", "hiker")))
        )
        active = _as_bool(data.get("active", True))
        event = {
            "type": "sos",
            "hiker_id": hiker_id,
            "active": active,
            "source": data.get("source", "unknown"),
            "stamp": self.get_clock().now().nanoseconds / 1e9,
        }
        with self._lock:
            self._merge_hiker_locked({"hiker_id": hiker_id, "sos_active": active})
            if active:
                self._sos_events.appendleft(event)
        self.get_logger().warn(f"SOS detected from {hiker_id}.")
        self._broadcast(event)

    def _parse_status_text(self, text: str) -> Optional[Dict]:
        match = _STATUS_RE.search(text)
        if not match:
            return None

        hiker_id = match["hiker_id"]
        payload = {
            "hiker_id": hiker_id,
            "trail": match["trail"],
            "x": _float_or_none(match["x"]),
            "y": _float_or_none(match["y"]),
            "terrain_z": _float_or_none(match["terrain_z"]),
            "altitude_m": _float_or_none(match["altitude"]),
            "gps": {
                "lat": _float_or_none(match["gps_lat"]),
                "lon": _float_or_none(match["gps_lon"]),
            },
            "battery_pct": _float_or_none(match["battery"]),
            "dop": _float_or_none(match["dop"]),
            "gps_error_m": _float_or_none(match["gps_err"]),
            "weather": match["weather"],
        }

        control_match = _CONTROL_RE.search(text)
        if control_match:
            payload["control"] = control_match["control"]
            payload["manual_control"] = control_match["control"] == "manual"

        sos_match = _SOS_RE.search(text)
        if sos_match:
            payload["sos_active"] = _as_bool(sos_match["sos"], False)

        sos_count_match = _SOS_COUNT_RE.search(text)
        if sos_count_match:
            payload["sos_count"] = int(sos_count_match["count"])

        return payload

    def _status_payload_from_json(self, data: Dict) -> Optional[Dict]:
        hiker_id = data.get("hiker_id")
        if not hiker_id:
            return None

        payload = {
            "hiker_id": str(hiker_id),
            "trail": data.get("trail_name") or data.get("trail"),
            "x": data.get("x"),
            "y": data.get("y"),
            "terrain_z": data.get("terrain_z"),
            "battery_pct": data.get("battery_pct"),
            "gps_error_m": data.get("gps_error_m"),
            "weather": data.get("weather"),
            "node_active": data.get("node_active"),
            "manual_control": data.get("manual_control"),
            "moving": data.get("moving"),
            "sos_active": data.get("sos_active"),
            "sos_count": data.get("sos_count"),
        }
        return {key: value for key, value in payload.items() if value is not None}

    def _store_hiker(self, payload: Dict) -> None:
        with self._lock:
            self._merge_hiker_locked(payload)

    def _merge_hiker_locked(self, payload: Dict) -> None:
        hiker_id = str(payload.get("hiker_id", "hiker"))
        current = dict(self._hikers.get(hiker_id, {"hiker_id": hiker_id}))
        current.update(payload)
        self._hikers[hiker_id] = current

    def destroy_node(self) -> None:
        try:
            self._server.shutdown()
            self._server.server_close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebDashboardNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
