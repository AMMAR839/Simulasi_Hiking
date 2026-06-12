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
    terrain_height_world,
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
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width,initial-scale=1"/>
  <title>SiLacak | Base Camp Dashboard</title>
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
  <style>
    :root{
      --bg:#0d1117;
      --panel:#161b22;
      --panel-2:#1c2230;
      --panel-3:#20293a;
      --line:rgba(148,168,196,.09);
      --line-2:rgba(148,168,196,.16);
      --line-3:rgba(148,168,196,.26);
      --text:#dde5f0;
      --text-2:#a8b8cc;
      --muted:#6a7d93;
      --blue:#4d8fd6;
      --blue-glow:rgba(77,143,214,.18);
      --teal:#3dab7e;
      --green:#5aaa5a;
      --amber:#c8883a;
      --red:#c45050;
      --violet:#8068c8;
      --radius:18px;
      --rs:12px;
      --lw:300px;
      --rw:348px;
      --gap:11px;
      --font:"Segoe UI",Inter,system-ui,-apple-system,sans-serif;
    }
    [data-theme="light"]{
      --bg:#eaeff6;
      --panel:rgba(255,255,255,.94);
      --panel-2:#ffffff;
      --panel-3:#f4f7fc;
      --line:rgba(50,80,120,.10);
      --line-2:rgba(50,80,120,.18);
      --line-3:rgba(50,80,120,.28);
      --text:#18273a;
      --text-2:#526070;
      --muted:#7a8fa8;
      --blue:#1c5fba;
      --blue-glow:rgba(28,95,186,.12);
      --teal:#0f7050;
      --green:#267026;
      --amber:#7a5010;
      --red:#922020;
      --violet:#4a339a;
    }
    *{box-sizing:border-box;margin:0;padding:0}
    html,body{height:100%;overflow:hidden;background:var(--bg);color:var(--text);font-family:var(--font);-webkit-font-smoothing:antialiased}
    .app{position:relative;width:100vw;height:100vh;overflow:hidden}
    #map{position:absolute;inset:0;z-index:0;background:var(--bg)}

    /* ── floating glass panels ── */
    .floating{
      position:absolute;z-index:20;
      border:1px solid var(--line-2);
      background:var(--panel);
    }

    /* ── sidebars ── */
    .sidebar{top:var(--gap);bottom:var(--gap);border-radius:var(--radius);display:flex;flex-direction:column;overflow:hidden}
    .sidebar.left{left:var(--gap);width:var(--lw)}
    .sidebar.right{right:var(--gap);width:var(--rw)}
    .sidebar.collapsed{width:40px!important;height:40px!important;top:var(--gap);bottom:auto;border-radius:13px}
    .sidebar.collapsed .ch{display:none!important}

    .sh{height:44px;min-height:44px;flex:0 0 auto;display:flex;align-items:center;gap:8px;padding:0 9px;border-bottom:1px solid var(--line)}
    .sidebar.collapsed .sh{height:100%;min-height:100%;border:0;padding:0;justify-content:center}
    .tbtn{width:28px;height:28px;border-radius:9px;border:1px solid var(--line-2);background:transparent;color:var(--muted);cursor:pointer;display:grid;place-items:center;flex-shrink:0;transition:color .12s,background .12s}
    .tbtn:hover{color:var(--text);background:var(--panel-2)}
    .tbtn svg{width:13px;height:13px;fill:none;stroke:currentColor;stroke-width:2.2;stroke-linecap:round;stroke-linejoin:round}
    .slbl{display:flex;flex-direction:column;gap:1px;flex:1;min-width:0}
    .slbl strong{font-size:9px;letter-spacing:.13em;text-transform:uppercase;font-weight:700}
    .slbl span{font-size:8.5px;color:var(--muted);letter-spacing:.10em;text-transform:uppercase;font-weight:600}

    .sb{min-height:0;flex:1;display:flex;flex-direction:column;gap:7px;padding:7px;overflow-y:auto;overflow-x:hidden;scrollbar-width:thin;scrollbar-color:var(--line-2) transparent}

    /* ── inner panels ── */
    .pnl{border:1px solid var(--line);border-radius:var(--rs);background:var(--panel-2);flex-shrink:0}
    .pnl-fill{flex:1;min-height:0;overflow:hidden;display:flex;flex-direction:column}
    .ph{display:flex;align-items:center;justify-content:space-between;gap:8px;padding:7px 10px 6px;border-bottom:1px solid var(--line)}
    .ph .tl{font-size:8.5px;font-weight:700;letter-spacing:.12em;text-transform:uppercase;color:var(--muted)}
    .ph .sv{font-size:9.5px;color:var(--text-2);font-weight:600}
    .pb{padding:7px 9px 9px}

    /* ── metrics ── */
    .mgrid{display:grid;grid-template-columns:1fr 1fr;gap:5px}
    .mc{border:1px solid var(--line);border-radius:9px;background:var(--panel-3);padding:7px 9px}
    .mc strong{display:block;font-size:16px;font-weight:700;letter-spacing:-.025em;line-height:1.05}
    .mc span{display:block;margin-top:2px;font-size:8px;color:var(--muted);text-transform:uppercase;letter-spacing:.09em;font-weight:700}
    .mc.alert strong{color:var(--red)}
    .mc.good strong{color:var(--green)}

    /* ── sparklines ── */
    .sgrid{display:grid;grid-template-columns:1fr 1fr;gap:5px}
    .sc{border:1px solid var(--line);border-radius:9px;background:var(--panel-3);padding:6px 7px 5px}
    .sc-h{display:flex;align-items:baseline;justify-content:space-between;gap:4px;margin-bottom:2px}
    .sc-h strong{font-size:8px;font-weight:700;color:var(--muted);letter-spacing:.10em;text-transform:uppercase}
    .sc-h span{font-size:10.5px;font-weight:700}
    .sc-w{position:relative;height:32px}
    .sc-w canvas{position:absolute;inset:0;width:100%!important;height:32px!important}

    /* ── EKG ── */
    .ekg-c{border:1px solid var(--line);border-radius:9px;background:var(--panel-3);padding:7px 8px}
    .ekg-h{display:flex;align-items:baseline;justify-content:space-between;margin-bottom:5px}
    .ekg-h strong{font-size:8px;font-weight:700;color:var(--muted);letter-spacing:.10em;text-transform:uppercase}
    .ekg-h span{font-size:10.5px;font-weight:700;color:var(--text)}
    .ekg-w{position:relative;height:68px;border-radius:6px;overflow:hidden;background:rgba(0,0,0,.18)}
    .ekg-w canvas{position:absolute;inset:0}

    /* ── rf params ── */
    .rf-row{display:grid;grid-template-columns:1fr 1fr;gap:5px;margin-bottom:7px}
    .rf-c{border:1px solid var(--line);border-radius:9px;background:var(--panel-3);padding:6px 8px}
    .rf-c strong{display:block;font-size:8px;color:var(--muted);letter-spacing:.10em;text-transform:uppercase;margin-bottom:3px}
    .rf-c span{display:block;font-size:10.5px;font-weight:700;line-height:1.3}
    .lgnd{display:flex;gap:8px;flex-wrap:wrap}
    .lg{display:flex;align-items:center;gap:4px;font-size:9px;color:var(--muted)}
    .sw.circle{width:7px;height:7px;border-radius:50%}
    .sw.box{width:7px;height:7px;border-radius:2px}
    .sw.tri{width:0;height:0;border-left:5px solid transparent;border-right:5px solid transparent;border-bottom:9px solid #d79025}
    .sw.line{width:12px;height:3px;border-radius:1px}
    .sw.terrain{width:18px;height:7px;border-radius:2px;background:linear-gradient(90deg,#4052aa,#24a6d0,#22c882,#eef08a,#9b7064,#faf8f2)}
    .terrain-contour{image-rendering:auto}

    /* ── tabs ── */
    .tabs{display:grid;grid-template-columns:repeat(4,1fr);gap:4px}
    .tab{height:30px;border-radius:8px;border:1px solid var(--line);background:var(--panel-3);color:var(--muted);font-weight:700;font-size:9px;cursor:pointer;letter-spacing:.05em;transition:all .12s}
    .tab.active{color:var(--text);background:var(--panel-2);border-color:var(--line-2)}
    .tab:hover:not(.active){color:var(--text-2)}

    /* ── selected card ── */
    .selc{border:1px solid var(--line);border-radius:9px;background:var(--panel-3);padding:8px 10px;min-height:80px}
    .selr{display:flex;align-items:center;justify-content:space-between;gap:6px;margin-bottom:4px}
    .selr strong{font-size:11.5px;font-weight:700;letter-spacing:.05em}
    .selb{font-size:9.5px;color:var(--text-2);line-height:1.65}
    .selb b{color:var(--text);font-weight:600}

    /* ── hiker grid ── */
    .hgrid{display:grid;grid-template-columns:repeat(2,1fr);gap:5px}
    .hcard{border:1px solid var(--line);border-radius:9px;background:var(--panel-3);padding:7px 8px;cursor:pointer;display:flex;flex-direction:column;gap:2px;transition:border-color .12s,background .12s;position:relative}
    .hcard::after{content:'';position:absolute;inset:0;border-radius:inherit;opacity:0;transition:opacity .12s;background:rgba(255,255,255,.04)}
    .hcard:hover::after{opacity:1}
    .hcard:hover{border-color:var(--line-2)}
    .hcard.sel{background:var(--blue-glow);border-color:rgba(77,143,214,.30)}
    .hcard.sos{background:rgba(196,80,80,.10);border-color:rgba(196,80,80,.28)}
    .hcard.sel.sos{background:rgba(196,80,80,.14);border-color:rgba(196,80,80,.35)}
    .ht{display:flex;align-items:center;justify-content:space-between;gap:5px}
    .hid{font-size:10.5px;font-weight:700;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
    .htr{font-size:8.5px;color:var(--muted);text-transform:capitalize}
    .hpos{font-size:8px;color:var(--muted);opacity:.8}
    .bwrap{display:flex;align-items:center;gap:5px;margin-top:1px}
    .bar{height:4px;flex:1;border-radius:999px;background:rgba(100,130,160,.12);overflow:hidden}
    .fill{height:100%;border-radius:inherit;transition:width .4s}
    .fill.hi{background:var(--green)}
    .fill.mid{background:var(--amber)}
    .fill.lo{background:var(--red)}
    .blbl{font-size:8px;color:var(--muted);min-width:24px;text-align:right}

    /* ── badges ── */
    .bdg{font-size:7.5px;font-weight:800;text-transform:uppercase;letter-spacing:.06em;padding:2px 5px;border-radius:999px;border:1px solid transparent;white-space:nowrap}
    .bdg.ok{background:rgba(90,170,90,.12);color:var(--green);border-color:rgba(90,170,90,.18)}
    .bdg.bad{background:rgba(196,80,80,.12);color:var(--red);border-color:rgba(196,80,80,.20)}
    .bdg.warn{background:rgba(200,136,58,.12);color:var(--amber);border-color:rgba(200,136,58,.18)}
    .bdg.info{background:var(--blue-glow);color:var(--blue);border-color:rgba(77,143,214,.22)}

    /* ── event/sos/link lists ── */
    .evl,.sosl,.lnkl{display:flex;flex-direction:column;gap:5px}
    .ev,.sosev,.lnk{border:1px solid var(--line);border-radius:9px;background:var(--panel-3);padding:6px 8px}
    .ev.ok{border-left:2px solid var(--green)}
    .ev.drop{border-left:2px solid var(--amber)}
    .sosev{border-left:2px solid var(--red);background:rgba(196,80,80,.07)}
    .lnk{border-left:2px solid var(--blue)}
    .etit,.ltit{display:flex;align-items:center;justify-content:space-between;gap:6px;font-size:9.5px;font-weight:700;margin-bottom:2px}
    .emeta,.lmeta,.smeta{font-size:8.5px;color:var(--muted);line-height:1.55}
    .empty{border:1px dashed var(--line-2);border-radius:9px;min-height:56px;display:flex;align-items:center;justify-content:center;color:var(--muted);font-size:9px}

    /* ── map controls (top-center pill) ── */
    .map-ctl{
      position:absolute;top:var(--gap);left:50%;transform:translateX(-50%);
      z-index:30;
      display:flex;align-items:center;gap:1px;
      background:var(--panel);
      border:1px solid var(--line-2);
      border-radius:12px;
      padding:3px;
    }
    .mbtnn{
      width:32px;height:32px;border-radius:9px;border:none;
      background:transparent;color:var(--text-2);cursor:pointer;
      display:grid;place-items:center;transition:background .12s,color .12s;
    }
    .mbtnn:hover{background:var(--panel-2);color:var(--text)}
    .mbtnn svg{width:15px;height:15px;fill:none;stroke:currentColor;stroke-width:2;stroke-linecap:round;stroke-linejoin:round}
    .msep{width:1px;height:20px;background:var(--line-2);flex-shrink:0;margin:0 1px}
    .mbtnn.fit-btn{font-size:9px;font-weight:700;letter-spacing:.06em;width:auto;padding:0 8px}

    /* ── conn indicator ── */
    .conn-dot{display:inline-block;width:6px;height:6px;border-radius:50%;background:var(--muted);margin-right:4px;transition:background .3s}
    .conn-dot.live{background:var(--green)}
    .conn-dot.err{background:var(--red)}
    .conn-dot.wait{background:var(--amber)}

    /* ── tooltip ── */
    .tt{
      position:fixed;z-index:9999;
      background:var(--panel-2);border:1px solid var(--line-2);
      border-radius:8px;padding:5px 9px;
      font-size:10px;line-height:1.5;color:var(--text);
      pointer-events:none;opacity:0;transition:opacity .12s;
      white-space:nowrap;max-width:220px;white-space:normal;
      box-shadow:0 6px 20px rgba(0,0,0,.28);
    }
    .tt.show{opacity:1}

    /* ── leaflet overrides ── */
    .leaflet-container{background:transparent!important}
    .leaflet-control-zoom{display:none!important}
    .leaflet-popup-content-wrapper,.leaflet-popup-tip{background:var(--panel-2)!important;color:var(--text)!important;border:1px solid var(--line-2)!important;font-family:var(--font)!important;box-shadow:0 8px 24px rgba(0,0,0,.3)!important}
    .leaflet-popup-content{font-family:var(--font)!important;font-size:11px!important;line-height:1.55!important}

    /* ── markers ── */
    .mk-h,.mk-b{display:grid;place-items:center;border:2px solid rgba(255,255,255,.85);transition:transform .15s}
    .mk-h{width:24px;height:24px;background:var(--blue);color:#fff;font-size:8.5px;font-weight:700;box-shadow:0 2px 8px rgba(0,0,0,.3);cursor:pointer;user-select:none}
    .mk-h.sel{width:32px;height:32px;font-size:11px;background:#1a3d6e;border-color:#fff;box-shadow:0 0 0 3px rgba(77,143,214,.35),0 4px 12px rgba(0,0,0,.4)}
    .mk-h.sos{background:var(--red);border-color:rgba(255,200,200,.8)}
    .mk-r{display:grid;place-items:center;width:21px;height:21px;color:#d79025;font-size:22px;line-height:21px;text-shadow:-1px 0 #4d3210,0 1px #4d3210,1px 0 #4d3210,0 -1px #4d3210,0 2px 5px rgba(0,0,0,.30)}
    .mk-b{width:18px;height:18px;border-radius:3px;background:#2d72bf;box-shadow:0 2px 8px rgba(0,0,0,.25)}
    .map-label{background:transparent;border:0;box-shadow:none;color:#1d2c36;font-size:11px;font-weight:600;text-shadow:0 1px 2px rgba(255,255,255,.72);pointer-events:none}
    .map-label.small{font-size:10px;font-weight:500}
    .map-label.base{color:#1f73c9;font-weight:700}

    @media(max-width:900px){
      html,body{overflow:auto}
      .app{height:auto;min-height:100vh}
      #map{position:relative;min-height:52vh}
      .sidebar{position:relative;left:auto!important;right:auto!important;top:auto;bottom:auto;width:auto!important;height:auto!important;margin:8px;border-radius:16px}
      .sidebar.collapsed{width:auto!important;height:auto!important}
      .map-ctl{top:56px}
    }
  </style>
</head>
<body>
<div class="app">
  <div id="map"></div>

  <!-- tooltip -->
  <div class="tt" id="tt"></div>

  <!-- LEFT sidebar -->
  <aside class="floating sidebar left" id="ls">
    <div class="sh">
      <button class="tbtn" id="lt" title="Collapse sidebar">
        <svg viewBox="0 0 24 24"><path d="M14 5l-7 7 7 7"/></svg>
      </button>
      <div class="slbl ch"><strong>workspace</strong><span>ringkasan &amp; sinyal</span></div>
      <span class="ch" style="margin-left:auto;display:flex;align-items:center;font-size:9px;color:var(--muted)">
        <span class="conn-dot" id="cdot"></span><span id="clbl">—</span>
      </span>
    </div>
    <div class="sb ch">
      <!-- ringkasan -->
      <div class="pnl">
        <div class="ph"><div class="tl">ringkasan</div><div class="sv" id="sum-st">waiting</div></div>
        <div class="pb">
          <div class="mgrid">
            <div class="mc" id="mc-hk"><strong id="mv-hk">0</strong><span>pendaki aktif</span></div>
            <div class="mc" id="mc-sos"><strong id="mv-sos">0</strong><span>sos aktif</span></div>
            <div class="mc"><strong id="mv-pdr">0%</strong><span>pdr sesi</span></div>
            <div class="mc"><strong id="mv-lat">—</strong><span>latency e2e</span></div>
          </div>
        </div>
      </div>

      <!-- sparklines -->
      <div class="pnl">
        <div class="ph"><div class="tl">signal trend</div><div class="sv" id="trend-scope">keseluruhan · 60 sampel terakhir</div></div>
        <div class="pb">
          <div class="sgrid">
            <div class="sc" data-tip="PDR — Packet Delivery Rate. Makin tinggi makin baik.">
              <div class="sc-h"><strong>pdr</strong><span id="sp-pdr-v">—</span></div>
              <div class="sc-w"><canvas id="sp-pdr"></canvas></div>
            </div>
            <div class="sc" data-tip="End-to-end latency dalam milidetik.">
              <div class="sc-h"><strong>latency</strong><span id="sp-lat-v">—</span></div>
              <div class="sc-w"><canvas id="sp-lat"></canvas></div>
            </div>
            <div class="sc" data-tip="Signal-to-Noise Ratio dB. Makin tinggi = sinyal lebih bersih.">
              <div class="sc-h"><strong>snr</strong><span id="sp-snr-v">—</span></div>
              <div class="sc-w"><canvas id="sp-snr"></canvas></div>
            </div>
            <div class="sc" data-tip="Link margin dB — jarak dari threshold. &lt; 0 berarti link putus.">
              <div class="sc-h"><strong>margin</strong><span id="sp-mrg-v">—</span></div>
              <div class="sc-w"><canvas id="sp-mrg"></canvas></div>
            </div>
          </div>
        </div>
      </div>

      <!-- EKG -->
      <div class="pnl">
        <div class="ph"><div class="tl">latency e2e</div><div class="sv" id="lat-now">— ms</div></div>
        <div class="pb" style="padding-bottom:8px">
          <div class="ekg-c">
            <div class="ekg-h"><strong>timeline rolling</strong><span id="lat-cap">—</span></div>
            <div class="ekg-w"><canvas id="ekg"></canvas></div>
          </div>
          <div style="display:flex;justify-content:space-between;margin-top:4px;font-size:8px;color:var(--muted)">
            <span>← 200 sampel</span>
            <span style="color:rgba(200,136,58,.8)">— target 30s</span>
            <span>terbaru →</span>
          </div>
        </div>
      </div>

      <!-- RF params -->
      <div class="pnl">
        <div class="ph"><div class="tl">rf params</div><div class="sv" id="rf-sh">—</div></div>
        <div class="pb">
          <div class="rf-row">
            <div class="rf-c" data-tip="Kondisi cuaca yang mempengaruhi propagasi sinyal RF"><strong>kondisi cuaca</strong><span id="rf-tag">—</span></div>
            <div class="rf-c" data-tip="Spreading Factor LoRa dan Time-on-Air paket"><strong>sf / toa</strong><span id="rf-line">—</span></div>
          </div>
            <div class="lgnd">
              <div class="lg"><span class="sw circle" style="background:var(--blue)"></span>hiker</div>
              <div class="lg"><span class="sw tri"></span>relay</div>
              <div class="lg"><span class="sw box" style="background:#2d72bf"></span>base</div>
              <div class="lg"><span class="sw line" style="background:var(--green)"></span>link kuat</div>
              <div class="lg"><span class="sw line" style="background:var(--amber)"></span>marginal</div>
              <div class="lg"><span class="sw line" style="background:var(--red)"></span>sos/drop</div>
              <div class="lg"><span class="sw terrain"></span>ketinggian</div>
            </div>
        </div>
      </div>
    </div>
  </aside>

  <!-- RIGHT sidebar -->
  <aside class="floating sidebar right" id="rs">
    <div class="sh">
      <button class="tbtn" id="rt" title="Collapse sidebar">
        <svg viewBox="0 0 24 24"><path d="M10 5l7 7-7 7"/></svg>
      </button>
      <div class="slbl ch"><strong>monitoring</strong><span>hiker, lora, sos</span></div>
    </div>
    <div class="sb ch">
      <div style="display:flex;flex-direction:column;gap:7px;min-height:0;flex:1">

        <!-- selected hiker -->
        <div class="pnl">
          <div class="ph"><div class="tl">selected hiker</div><div class="sv" id="sel-lbl">none</div></div>
          <div class="pb">
            <div id="sel-card" class="selc">
              <div class="selr"><strong>—</strong><span style="font-size:9px;color:var(--muted)">klik pendaki untuk detail</span></div>
              <div class="selb">Pilih pendaki dari daftar atau klik marker di peta.</div>
            </div>
          </div>
        </div>

        <!-- tabs row -->
        <div class="pnl" style="flex-shrink:0">
          <div class="pb" style="padding:5px 7px">
            <div class="tabs">
              <button class="tab active" data-tab="hikers">hikers</button>
              <button class="tab" data-tab="links">links</button>
              <button class="tab" data-tab="events">events</button>
              <button class="tab" data-tab="sos">sos</button>
            </div>
          </div>
        </div>

        <!-- tab: hikers -->
        <div class="pnl pnl-fill tab-panel" id="tp-hikers">
          <div class="ph"><div class="tl">daftar pendaki</div><div class="sv" id="hk-cnt">0</div></div>
          <div class="pb" style="overflow-y:auto;flex:1;min-height:0">
            <div id="hk-list" class="hgrid"><div class="empty" style="grid-column:1/-1">menunggu data…</div></div>
          </div>
        </div>

        <!-- tab: links -->
        <div class="pnl pnl-fill tab-panel" id="tp-links" style="display:none">
          <div class="ph"><div class="tl">link lora aktif</div><div class="sv" id="lnk-ctx">—</div></div>
          <div class="pb" style="overflow-y:auto;flex:1;min-height:0">
            <div id="lnk-list" class="lnkl"><div class="empty">klik pendaki → lihat link</div></div>
          </div>
        </div>

        <!-- tab: events -->
        <div class="pnl pnl-fill tab-panel" id="tp-events" style="display:none">
          <div class="ph"><div class="tl">event jaringan</div><div class="sv" id="ev-cnt">0</div></div>
          <div class="pb" style="overflow-y:auto;flex:1;min-height:0">
            <div id="ev-list" class="evl"><div class="empty">menunggu event…</div></div>
          </div>
        </div>

        <!-- tab: sos -->
        <div class="pnl pnl-fill tab-panel" id="tp-sos" style="display:none">
          <div class="ph"><div class="tl">sos aktif</div><div class="sv" id="sos-cnt">0</div></div>
          <div class="pb" style="overflow-y:auto;flex:1;min-height:0">
            <div id="sos-list" class="sosl"><div class="empty">belum ada sos 👍</div></div>
          </div>
        </div>

      </div>
    </div>
  </aside>

  <!-- MAP CONTROLS — pill at top center -->
  <div class="map-ctl floating">
    <button class="mbtnn" id="mc-zi" title="Zoom in (Ctrl +)">
      <svg viewBox="0 0 24 24"><circle cx="11" cy="11" r="8"/><line x1="21" y1="21" x2="16.65" y2="16.65"/><line x1="11" y1="8" x2="11" y2="14"/><line x1="8" y1="11" x2="14" y2="11"/></svg>
    </button>
    <button class="mbtnn" id="mc-zo" title="Zoom out (Ctrl -)">
      <svg viewBox="0 0 24 24"><circle cx="11" cy="11" r="8"/><line x1="21" y1="21" x2="16.65" y2="16.65"/><line x1="8" y1="11" x2="14" y2="11"/></svg>
    </button>
    <div class="msep"></div>
    <button class="mbtnn fit-btn" id="mc-fit" title="Fit semua ke layar">FIT</button>
    <div class="msep"></div>
    <button class="mbtnn" id="mc-theme" title="Ganti tema terang/gelap">
      <svg viewBox="0 0 24 24"><path d="M21 12.8A9 9 0 1 1 11.2 3a7 7 0 0 0 9.8 9.8z"/></svg>
    </button>
  </div>

</div><!-- .app -->

<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
(() => {
'use strict';

/* ══════════════════════════════════════
   STATE
══════════════════════════════════════ */
const S = {
  scenario: null, hikers: {}, selectedId: null,
  stats: { pdr_pct:0 },
  events: [], sosEvents: [],
  firstFit: false, _sceneDone: false,
};
const H = { overall: makeTrend(), byHiker: {} };

function makeTrend() {
  return { pdr: [], lat: [], snr: [], mrg: [], total: 0, delivered: 0 };
}

function hikerTrend(hikerId) {
  if (!H.byHiker[hikerId]) H.byHiker[hikerId] = makeTrend();
  return H.byHiker[hikerId];
}

function activeTrend() {
  return S.selectedId ? hikerTrend(S.selectedId) : H.overall;
}

// ── debounced render: coalesce rapid WS updates into one rAF ──
let _renderPending = false;
function scheduleRender() {
  if (_renderPending) return;
  _renderPending = true;
  requestAnimationFrame(() => { _renderPending = false; renderAll(); });
}

/* ══════════════════════════════════════
   EKG  —  rolling smoothed buffer
══════════════════════════════════════ */
const EKG_N  = 200;
const ekgRaw = [];
const ekgSm  = [];
let ekgRaf   = null;
let ekgFps   = 0;

function ekgInit() {
  // seed with mid-range baseline so graph starts with visible height
  const base = 0.015;
  let sm = base;
  for (let i = 0; i < EKG_N; i++) {
    const noise = (Math.random() - 0.5) * 0.004;
    sm = Math.max(0.005, Math.min(0.08, sm + noise));
    ekgRaw.push(sm * 60000);
    ekgSm.push(sm);
  }
}

function ekgPush(ms) {
  const v = Math.min(Math.max(ms, 0), 60000);
  ekgRaw.push(v);
  if (ekgRaw.length > EKG_N) ekgRaw.shift();

  const raw01 = v / 60000;
  const prev  = ekgSm.length ? ekgSm[ekgSm.length - 1] : raw01;
  const alpha = 0.18;   // smoothed — less jumpy
  ekgSm.push(prev * (1 - alpha) + raw01 * alpha);
  if (ekgSm.length > EKG_N) ekgSm.shift();
}

function ekgDraw() {
  const canvas = document.getElementById('ekg');
  if (!canvas) return;
  const par = canvas.parentElement;
  const W = par.clientWidth, H2 = par.clientHeight;
  if (!W || !H2) return;
  const dpr = window.devicePixelRatio || 1;
  if (canvas.width !== Math.floor(W*dpr)) {
    canvas.width  = Math.floor(W * dpr);
    canvas.height = Math.floor(H2 * dpr);
    canvas.style.width  = W + 'px';
    canvas.style.height = H2 + 'px';
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, W, H2);

  const pad = 4;
  const ch  = H2 - pad * 2;

  // subtle grid
  ctx.save();
  ctx.strokeStyle = 'rgba(77,143,214,.06)';
  ctx.setLineDash([2, 6]);
  for (let i = 1; i < 4; i++) {
    const y = pad + (i / 4) * ch;
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(W, y); ctx.stroke();
  }
  for (let i = 1; i < 8; i++) {
    const x = (i / 8) * W;
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, H2); ctx.stroke();
  }
  ctx.restore();

  if (ekgSm.length < 2) return;

  // ── dynamic range: floor with 25% padding below min, 10% above max ──
  const rawMin = Math.min(...ekgSm);
  const rawMax = Math.max(...ekgSm);
  const range  = rawMax - rawMin || 0.01;
  const floor  = Math.max(0, rawMin - range * 0.25);
  const ceil   = Math.min(1, rawMax + range * 0.10);
  const span   = ceil - floor || 0.01;
  const yf     = v => pad + (1 - (v - floor) / span) * ch;

  // target line at 30 000 ms = 0.5 in 0-1 space; only draw if visible in range
  const targetV = 0.5;
  if (targetV >= floor && targetV <= ceil) {
    ctx.save();
    ctx.strokeStyle = 'rgba(200,136,58,.28)';
    ctx.lineWidth = .8;
    ctx.setLineDash([3, 6]);
    const ty = yf(targetV);
    ctx.beginPath(); ctx.moveTo(0, ty); ctx.lineTo(W, ty); ctx.stroke();
    ctx.restore();
  }

  const step = W / (EKG_N - 1);
  const blueColor = getCSS('--blue') || '#4d8fd6';

  // area fill
  ctx.beginPath();
  ekgSm.forEach((v, i) => {
    const x = i * step, y = yf(v);
    i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
  });
  ctx.lineTo((EKG_N-1)*step, H2);
  ctx.lineTo(0, H2);
  ctx.closePath();
  ctx.globalAlpha = .09;
  ctx.fillStyle = blueColor;
  ctx.fill();
  ctx.globalAlpha = 1;

  // soft glow pass
  ctx.save();
  ctx.shadowColor = 'rgba(77,143,214,.5)';
  ctx.shadowBlur  = 5;
  ctx.strokeStyle = 'rgba(77,143,214,.25)';
  ctx.lineWidth   = 2.5;
  ctx.lineJoin    = 'round';
  ctx.beginPath();
  ekgSm.forEach((v, i) => {
    const x = i * step, y = yf(v);
    i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
  });
  ctx.stroke();
  ctx.restore();

  // crisp main line
  ctx.strokeStyle = blueColor;
  ctx.lineWidth   = 1.4;
  ctx.lineJoin    = 'round';
  ctx.beginPath();
  ekgSm.forEach((v, i) => {
    const x = i * step, y = yf(v);
    i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
  });
  ctx.stroke();

  // leading dot with pulse
  const last = ekgSm[ekgSm.length - 1];
  const lx = (EKG_N-1) * step, ly = yf(last);
  const pulse = 0.5 + 0.5 * Math.sin(ekgFps * 0.08);
  ctx.save();
  ctx.shadowColor = blueColor;
  ctx.shadowBlur  = 6 + pulse * 4;
  ctx.fillStyle   = blueColor;
  ctx.globalAlpha = 0.3 + pulse * 0.3;
  ctx.beginPath(); ctx.arc(lx, ly, 5, 0, Math.PI*2); ctx.fill();
  ctx.globalAlpha = 1;
  ctx.beginPath(); ctx.arc(lx, ly, 2.5, 0, Math.PI*2); ctx.fill();
  ctx.restore();
}

function ekgLoop(ts) {
  ekgFps++;
  ekgDraw();
  ekgRaf = requestAnimationFrame(ekgLoop);
}
function ekgStart() { if (!ekgRaf) ekgRaf = requestAnimationFrame(ekgLoop); }

/* ══════════════════════════════════════
   SPARKLINES
══════════════════════════════════════ */
function spark(canvas, data, color, refVal) {
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const dpr = window.devicePixelRatio || 1;
  const W   = canvas.parentElement.clientWidth || 100, Hs = 32;
  canvas.width  = Math.floor(W * dpr);
  canvas.height = Math.floor(Hs * dpr);
  canvas.style.width  = W  + 'px';
  canvas.style.height = Hs + 'px';
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, W, Hs);

  if (!data || data.length < 2) {
    ctx.fillStyle = 'rgba(100,130,160,.07)';
    ctx.fillRect(0, Hs/2 - .5, W, 1);
    return;
  }
  const mn = Math.min(...data), mx = Math.max(...data), rng = mx - mn || 1;
  const yf = v => Hs - 2 - ((v - mn) / rng) * (Hs - 4);
  const st = W / (data.length - 1);

  if (refVal !== undefined) {
    ctx.save();
    ctx.strokeStyle = 'rgba(196,80,80,.22)';
    ctx.setLineDash([2, 4]);
    ctx.beginPath(); ctx.moveTo(0, yf(refVal)); ctx.lineTo(W, yf(refVal)); ctx.stroke();
    ctx.restore();
  }

  ctx.beginPath();
  data.forEach((v, i) => { const x=i*st, y=yf(v); i===0?ctx.moveTo(x,y):ctx.lineTo(x,y); });
  ctx.lineTo((data.length-1)*st, Hs); ctx.lineTo(0, Hs); ctx.closePath();
  ctx.globalAlpha = .10; ctx.fillStyle = color; ctx.fill(); ctx.globalAlpha = 1;

  ctx.strokeStyle = color; ctx.lineWidth = 1.3; ctx.lineJoin = 'round';
  ctx.beginPath();
  data.forEach((v, i) => { const x=i*st, y=yf(v); i===0?ctx.moveTo(x,y):ctx.lineTo(x,y); });
  ctx.stroke();

  const lv = data[data.length-1];
  ctx.fillStyle = color;
  ctx.beginPath(); ctx.arc((data.length-1)*st, yf(lv), 2, 0, Math.PI*2); ctx.fill();
}

function redrawSparks() {
  const t = activeTrend();
  spark(document.getElementById('sp-pdr'), t.pdr, getCSS('--green')||'#5aaa5a', 80);
  spark(document.getElementById('sp-lat'), t.lat, getCSS('--teal')||'#3dab7e');
  spark(document.getElementById('sp-snr'), t.snr, getCSS('--blue')||'#4d8fd6');
  spark(document.getElementById('sp-mrg'), t.mrg, getCSS('--amber')||'#c8883a');
}

/* ══════════════════════════════════════
   UTIL
══════════════════════════════════════ */
function getCSS(k) { return getComputedStyle(document.documentElement).getPropertyValue(k).trim(); }
function fmt(v, d=1) { const n=Number(v); return Number.isFinite(n)?n.toFixed(d):'-'; }
function sid(id) { const p=String(id||'?').split('_'); return p.length>1?p[1]:p[0]; }
function xy(x,y) { return [Number(y), Number(x)]; }
function bcls(p) { const n=Number(p); return !Number.isFinite(n)?'mid':n>=60?'hi':n>=25?'mid':'lo'; }
function push60(arr, v) { const n=Number(v); if(!Number.isFinite(n))return; arr.push(n); if(arr.length>60)arr.shift(); }
function lastVal(arr) { return arr && arr.length ? arr[arr.length - 1] : null; }
function trendEventMargin(ev) {
  const margins = Array.isArray(ev?.links)
    ? ev.links.map(l => Number(l.margin_db)).filter(Number.isFinite)
    : [];
  if (margins.length) return Math.min(...margins);
  const m = Number(ev?.margin_db);
  return Number.isFinite(m) ? m : null;
}
function trendEventSnr(ev) {
  const s = Number(ev?.snr_db);
  if (Number.isFinite(s)) return s;
  const snrs = Array.isArray(ev?.links)
    ? ev.links.map(l => Number(l.snr_db)).filter(Number.isFinite)
    : [];
  return snrs.length ? Math.min(...snrs) : null;
}
function addTrendEvent(trend, ev, pdrOverride) {
  if (!trend || !ev) return;
  trend.total += 1;
  if (ev.delivered) trend.delivered += 1;
  const pdr = Number.isFinite(Number(pdrOverride))
    ? Number(pdrOverride)
    : (trend.total ? trend.delivered / trend.total * 100.0 : 0.0);
  push60(trend.pdr, pdr);
  push60(trend.lat, ev.end_to_end_latency_ms);
  push60(trend.snr, trendEventSnr(ev));
  push60(trend.mrg, trendEventMargin(ev));
}
function resetTrends() {
  H.overall = makeTrend();
  H.byHiker = {};
}
function rebuildTrends(events) {
  resetTrends();
  const ordered = Array.isArray(events) ? [...events].reverse() : [];
  ordered.forEach(ev => {
    if (!ev || !ev.hiker_id) return;
    addTrendEvent(H.overall, ev);
    addTrendEvent(hikerTrend(ev.hiker_id), ev);
  });
}
const wLabel = { clear:'Cerah ☀', fog:'Kabut 🌫', light_rain:'Hujan ringan 🌦', heavy_rain:'Hujan lebat 🌧', thunderstorm:'Badai ⛈' };
const routeCol = { ridge_route:'#e46e1d', valley_route:'#2f6fd6', crater_route:'#6b42c7' };
const terrainBins = [
  [0.035, [70, 80, 170]],
  [0.090, [58, 111, 199]],
  [0.145, [42, 147, 220]],
  [0.200, [32, 174, 211]],
  [0.260, [31, 190, 181]],
  [0.320, [33, 203, 139]],
  [0.385, [67, 212, 111]],
  [0.450, [119, 220, 105]],
  [0.515, [176, 225, 116]],
  [0.580, [226, 231, 128]],
  [0.645, [238, 220, 142]],
  [0.705, [218, 193, 134]],
  [0.765, [197, 164, 120]],
  [0.825, [170, 132, 105]],
  [0.875, [145, 109, 101]],
  [0.925, [180, 165, 158]],
  [0.970, [218, 209, 204]],
  [1.010, [250, 248, 242]],
];

function terrainRgb(v, mn, mx) {
  const span = Math.max(0.001, Number(mx) - Number(mn));
  const t = Math.max(0, Math.min(1, (Number(v) - Number(mn)) / span));
  for (const [limit, rgb] of terrainBins) {
    if (t <= limit) return rgb;
  }
  return [233, 226, 209];
}

function _terrainVal(vals, cols, rows, x, y) {
  const xx = Math.max(0, Math.min(cols - 1, x));
  const yy = Math.max(0, Math.min(rows - 1, y));
  return Number(vals[(rows - 1 - yy) * cols + xx]);
}

function _edgeCross(a, b, level, ax, ay, bx, by) {
  if ((a < level && b < level) || (a > level && b > level) || a === b) return null;
  const t = (level - a) / (b - a);
  return [ax + (bx - ax) * t, ay + (by - ay) * t];
}

function drawContourLines(ctx, vals, cols, rows, minH, maxH) {
  const levels = Array.from({ length: 30 }, (_, i) => minH + (maxH - minH) * ((i + 1) / 31));
  ctx.save();
  ctx.strokeStyle = 'rgba(58,57,50,.23)';
  ctx.lineWidth = 0.22;
  ctx.lineJoin = 'round';
  ctx.lineCap = 'round';

  levels.forEach(level => {
    ctx.beginPath();
    for (let y = 0; y < rows - 1; y++) {
      for (let x = 0; x < cols - 1; x++) {
        const tl = _terrainVal(vals, cols, rows, x, y);
        const tr = _terrainVal(vals, cols, rows, x + 1, y);
        const br = _terrainVal(vals, cols, rows, x + 1, y + 1);
        const bl = _terrainVal(vals, cols, rows, x, y + 1);
        const pts = [
          _edgeCross(tl, tr, level, x, y, x + 1, y),
          _edgeCross(tr, br, level, x + 1, y, x + 1, y + 1),
          _edgeCross(br, bl, level, x + 1, y + 1, x, y + 1),
          _edgeCross(bl, tl, level, x, y + 1, x, y),
        ].filter(Boolean);
        if (pts.length >= 2) {
          ctx.moveTo(pts[0][0], pts[0][1]);
          ctx.lineTo(pts[1][0], pts[1][1]);
        }
        if (pts.length === 4) {
          ctx.moveTo(pts[2][0], pts[2][1]);
          ctx.lineTo(pts[3][0], pts[3][1]);
        }
      }
    }
    ctx.stroke();
  });
  ctx.restore();
}

function drawTerrainLayer(terrain, bounds) {
  const cols = Number(terrain?.cols || 0);
  const rows = Number(terrain?.rows || 0);
  const vals = terrain?.values || [];
  if (!cols || !rows || vals.length !== cols * rows) return;

  const scale = 2;
  const canvas = document.createElement('canvas');
  canvas.width = cols * scale;
  canvas.height = rows * scale;
  const ctx = canvas.getContext('2d');
  const minH = Number(terrain.min_h);
  const maxH = Number(terrain.max_h);

  ctx.imageSmoothingEnabled = false;
  for (let py = 0; py < rows; py++) {
    const srcY = rows - 1 - py;
    for (let x = 0; x < cols; x++) {
      const h = Number(vals[srcY * cols + x]);
      const rgb = terrainRgb(h, minH, maxH);
      ctx.fillStyle = `rgb(${rgb[0]},${rgb[1]},${rgb[2]})`;
      ctx.fillRect(x * scale, py * scale, scale, scale);
    }
  }

  ctx.save();
  ctx.scale(scale, scale);
  drawContourLines(ctx, vals, cols, rows, minH, maxH);
  ctx.restore();

  L.imageOverlay(canvas.toDataURL('image/png'), bounds, {
    opacity: 0.80,
    interactive: false,
    className: 'terrain-contour'
  }).addTo(ML.scene);
}

function labelHtml(text, cls='') {
  return L.divIcon({
    className: `map-label ${cls}`,
    html: String(text || '').replace(/_/g, '_'),
    iconSize: [96, 28],
    iconAnchor: [0, 12]
  });
}

function addMapLabel(x, y, text, cls='', dx=2.5, dy=2.5) {
  L.marker(xy(Number(x) + dx, Number(y) + dy), {
    icon: labelHtml(text, cls),
    interactive: false,
    keyboard: false
  }).addTo(ML.scene);
}

function obstacleStyle(obs) {
  const kind = String(obs.kind || '').toLowerCase();
  if (kind.includes('tree')) {
    return { color:'rgba(77,145,93,.55)', fillColor:'rgba(86,166,105,.28)' };
  }
  if (kind.includes('rock')) {
    return { color:'rgba(113,103,91,.50)', fillColor:'rgba(135,124,108,.22)' };
  }
  if (kind.includes('crater') || kind.includes('terrain')) {
    return { color:'rgba(126,82,196,.48)', fillColor:'rgba(126,82,196,.18)' };
  }
  return { color:'rgba(200,136,58,.55)', fillColor:'rgba(200,136,58,.18)' };
}

function obstacleLabel(name) {
  const parts = String(name || '').split('_');
  if (parts.length <= 2) return parts.join('_');
  return parts.join('<br>');
}

/* ══════════════════════════════════════
   MAP
══════════════════════════════════════ */
const map = L.map('map', { crs: L.CRS.Simple, minZoom: -4, maxZoom: 6, zoomControl: false });
const ML  = { scene: L.layerGroup().addTo(map), hikers: L.layerGroup().addTo(map), links: L.layerGroup().addTo(map) };
let sceneBounds = null;
map.setView([0, 0], 0);

// keyboard shortcuts
document.addEventListener('keydown', e => {
  if (e.ctrlKey && e.key === '=') { e.preventDefault(); map.zoomIn(); }
  if (e.ctrlKey && e.key === '-') { e.preventDefault(); map.zoomOut(); }
  if (e.key === 'Escape') { S.selectedId = null; renderAll(); }
});

document.getElementById('mc-zi').onclick  = () => map.zoomIn();
document.getElementById('mc-zo').onclick  = () => map.zoomOut();
document.getElementById('mc-fit').onclick = () => { if (sceneBounds) map.fitBounds(sceneBounds, { paddingTopLeft: [24,56], paddingBottomRight: [24,24] }); };
document.getElementById('mc-theme').onclick = () => {
  const t = document.documentElement.getAttribute('data-theme') === 'light' ? 'dark' : 'light';
  document.documentElement.setAttribute('data-theme', t);
  localStorage.setItem('sl-theme', t);
  redrawSparks();
};
document.documentElement.setAttribute('data-theme', localStorage.getItem('sl-theme') || 'dark');

function toggleSelectedHiker(hikerId) {
  S.selectedId = S.selectedId === hikerId ? null : hikerId;
  renderAll();
}

/* ══════════════════════════════════════
   TOOLTIP
══════════════════════════════════════ */
const ttEl = document.getElementById('tt');
let ttHide = null;
function showTip(e, html) {
  clearTimeout(ttHide);
  ttEl.innerHTML = html;
  ttEl.classList.add('show');
  moveTip(e);
}
function moveTip(e) {
  const x = e.clientX + 14, y = e.clientY + 10;
  ttEl.style.left = Math.min(x, window.innerWidth  - ttEl.offsetWidth  - 8) + 'px';
  ttEl.style.top  = Math.min(y, window.innerHeight - ttEl.offsetHeight - 8) + 'px';
}
function hideTip() { ttHide = setTimeout(() => ttEl.classList.remove('show'), 80); }

document.addEventListener('mouseover', e => {
  const el = e.target.closest('[data-tip]');
  if (el) showTip(e, el.dataset.tip);
});
document.addEventListener('mousemove', e => { if (ttEl.classList.contains('show')) moveTip(e); });
document.addEventListener('mouseout',  e => { if (e.target.closest('[data-tip]')) hideTip(); });

/* ══════════════════════════════════════
   SCENARIO RENDER
══════════════════════════════════════ */
function renderScene() {
  if (!S.scenario) return;
  ML.scene.clearLayers();
  const ext = Number(S.scenario.extent || 120);
  sceneBounds = [[-ext, -ext], [ext, ext]];

  const terrain = S.scenario.terrain || null;
  if (terrain) drawTerrainLayer(terrain, sceneBounds);

  L.rectangle(sceneBounds, { color: 'rgba(77,143,214,.3)', weight: .7, fillOpacity: .02 }).addTo(ML.scene);

  Object.entries(S.scenario.trails || {}).forEach(([name, pts]) => {
    const color = routeCol[name] || '#4d8fd6';
    L.polyline(pts.map(p => xy(p[0], p[1])), {
      color, weight: 3.6, opacity: .95
    }).bindTooltip(`<b>${name.replace(/_/g,' ')}</b>`, { sticky: true }).addTo(ML.scene);
    pts.forEach(p => {
      L.circleMarker(xy(p[0], p[1]), {
        radius: 2.2,
        color,
        weight: 0,
        fillColor: color,
        fillOpacity: 0.9,
        interactive: false
      }).addTo(ML.scene);
    });
  });

  (S.scenario.obstacles || []).forEach(obs => {
    const st = obstacleStyle(obs);
    L.circle(xy(obs.x, obs.y), {
      radius: Number(obs.radius),
      color: st.color,
      fillColor: st.fillColor,
      weight: 1.2,
      fillOpacity: 1.0
    }).bindTooltip(`<b>${obs.name}</b><br>${obs.kind} — loss ${obs.loss_db} dB`, { sticky: true }).addTo(ML.scene);
    addMapLabel(obs.x, obs.y, obstacleLabel(obs.name), 'small', -4, 2);
  });

  (S.scenario.lora_nodes || []).forEach(node => {
    L.marker(xy(node.x, node.y), {
      icon: L.divIcon({ className: '', html: '<div class="mk-r">▲</div>', iconSize: [22,22], iconAnchor: [11,15] })
    }).addTo(ML.scene).bindTooltip(`<b>${node.name}</b><br>Relay node`, { sticky: true });
    addMapLabel(node.x, node.y, node.name, '', 2.5, 2.5);
  });

  if (S.scenario.base_station) {
    const b = S.scenario.base_station;
    L.marker(xy(b.x, b.y), {
      icon: L.divIcon({ className: '', html: '<div class="mk-b"></div>', iconSize: [18,18], iconAnchor: [9,9] })
    }).addTo(ML.scene).bindTooltip(`<b>${b.name}</b><br>Base station`, { sticky: true });
    addMapLabel(b.x, b.y, b.name || 'base', 'base', 2.5, -5.0);
  }

  if (!S.firstFit) {
    map.fitBounds(sceneBounds, { paddingTopLeft: [24,56], paddingBottomRight: [24,24] });
    S.firstFit = true;
  }
}

/* ══════════════════════════════════════
   HIKER MARKERS — persistent, diff-updated (no flicker)
══════════════════════════════════════ */
const _mkCache = {};   // hiker_id → { marker, visualKey }

function _mkVisualKey(h) {
  // Only values that change marker appearance, not position.
  return `${h.sos_active?1:0},${S.selectedId===h.hiker_id?1:0}`;
}

function _mkIcon(h) {
  const isSel = S.selectedId === h.hiker_id;
  const cls   = `mk-h${isSel ? ' sel' : ''}${h.sos_active ? ' sos' : ''}`;
  const sz    = isSel ? 32 : 24;
  const hitSz = sz + 8;
  return L.divIcon({
    className: '',
    html: `<div style="width:${hitSz}px;height:${hitSz}px;display:flex;align-items:center;justify-content:center"><div class="${cls}" style="width:${sz}px;height:${sz}px;flex-shrink:0">${sid(h.hiker_id)}</div></div>`,
    iconSize:   [hitSz, hitSz],
    iconAnchor: [hitSz / 2, hitSz / 2]
  });
}

function syncMarkers(hikers) {
  const seen = new Set();
  hikers.forEach(h => {
    if (h.x === undefined || h.y === undefined) return;
    seen.add(h.hiker_id);
    const visualKey = _mkVisualKey(h);
    const cached = _mkCache[h.hiker_id];

    if (cached) {
      cached.marker.setLatLng(xy(h.x, h.y));
      if (cached.visualKey !== visualKey) {
        cached.marker.setIcon(_mkIcon(h));
        cached.marker.setZIndexOffset(S.selectedId === h.hiker_id ? 1000 : 500);
        cached.visualKey = visualKey;
      }
      return;
    }

    const mk = L.marker(xy(h.x, h.y), {
      icon: _mkIcon(h),
      interactive: true,
      zIndexOffset: S.selectedId === h.hiker_id ? 1000 : 500
    }).addTo(ML.hikers);

    mk.bindTooltip(
      () => `<b>${h.hiker_id}</b>${S.hikers[h.hiker_id]?.sos_active ? ' 🆘' : ''}<br>${(S.hikers[h.hiker_id]?.trail||'').replace(/_/g,' ')}<br>bat ${S.hikers[h.hiker_id]?.battery_pct ?? '-'}%`,
      { sticky: true, opacity: 0.92 }
    );

    mk.on('click', () => {
      toggleSelectedHiker(h.hiker_id);
    });

    _mkCache[h.hiker_id] = { marker: mk, visualKey };
  });

  // remove markers for hikers no longer present
  Object.keys(_mkCache).forEach(id => {
    if (!seen.has(id)) {
      ML.hikers.removeLayer(_mkCache[id].marker);
      delete _mkCache[id];
    }
  });
}

/* ══════════════════════════════════════
   HIKER RENDER
══════════════════════════════════════ */
function renderHikers() {
  const hikers = Object.values(S.hikers || {}).sort((a,b) => String(a.hiker_id).localeCompare(String(b.hiker_id)));
  const sosCnt = hikers.filter(h => h.sos_active).length;

  document.getElementById('hk-cnt').textContent = String(hikers.length);
  document.getElementById('mv-hk').textContent  = String(hikers.length);
  document.getElementById('mv-sos').textContent = String(sosCnt);
  document.getElementById('mc-sos').classList.toggle('alert', sosCnt > 0);

  const listEl = document.getElementById('hk-list');

  if (!hikers.length) {
    listEl.innerHTML = '<div class="empty" style="grid-column:1/-1">menunggu data pendaki…</div>';
    syncMarkers([]);
    return;
  }

  // Bersihin state "empty" kalau ada
  if (listEl.querySelector('.empty')) {
    listEl.innerHTML = '';
  }

  // Set ID yang aktif sekarang
  const currentIds = new Set(hikers.map(h => h.hiker_id));
  if (S.selectedId && !currentIds.has(S.selectedId)) S.selectedId = null;

  // Hapus card yang pendakinya udah gak ada di data
  Array.from(listEl.children).forEach(child => {
    if (!currentIds.has(child.dataset.hid)) {
      child.remove();
    }
  });

  // Update card yang ada atau bikin baru
  hikers.forEach(h => {
    const p   = Number(h.battery_pct);
    const bat = Number.isFinite(p) ? fmt(p, 0) + '%' : '-';
    const isSel = S.selectedId === h.hiker_id;
    const pos   = h.x !== undefined ? `${fmt(h.x,1)}, ${fmt(h.y,1)}` : '—';
    const cls = `hcard${isSel?' sel':''}${h.sos_active?' sos':''}`;

    let card = listEl.querySelector(`.hcard[data-hid="${h.hiker_id}"]`);

    if (!card) {
      card = document.createElement('div');
      card.dataset.hid = h.hiker_id;
      listEl.appendChild(card);
    }

    if (card.className !== cls) card.className = cls;

    // Bikin string HTML untuk konten dalam card
    const inner = `
      <div class="ht"><span class="hid">${h.hiker_id}</span><span class="bdg ${h.sos_active?'bad':'ok'}">${h.sos_active?'sos':'ok'}</span></div>
      <div class="htr">${(h.trail||'').replace(/_/g,' ')}</div>
      <div class="hpos">${pos}</div>
      <div class="bwrap"><div class="bar"><div class="fill ${bcls(p)}" style="width:${Number.isFinite(p)?Math.max(4,Math.min(100,p)):0}%"></div></div><span class="blbl">${bat}</span></div>
    `;

    // Cuma timpa innerHTML kalau emang isinya beda, biar ga thrashing DOM
    if (card.innerHTML !== inner) {
      card.innerHTML = inner;
    }
  });

  syncMarkers(hikers);
}

/* ══════════════════════════════════════
   SELECTED HIKER CARD
══════════════════════════════════════ */
function renderSelected() {
  const h = S.selectedId ? S.hikers[S.selectedId] : null;
  document.getElementById('sel-lbl').textContent = S.selectedId || 'none';

  if (!h) {
    document.getElementById('sel-card').innerHTML = `<div class="selr"><strong>—</strong><span style="font-size:9px;color:var(--muted)">klik pendaki untuk detail</span></div><div class="selb">Pilih pendaki dari daftar atau klik marker di peta.</div>`;
    document.getElementById('lnk-ctx').textContent = 'keseluruhan';
    return;
  }

  const sosHtml = h.sos_active ? `<span class="bdg bad">🆘 sos</span>` : `<span class="bdg ok">normal</span>`;
  const pos = h.x !== undefined ? `${fmt(h.x,2)}, ${fmt(h.y,2)}` : '—';
  document.getElementById('sel-card').innerHTML = `
    <div class="selr"><strong>${h.hiker_id}</strong>${sosHtml}</div>
    <div class="selb">
      <b>trail</b> ${(h.trail||'—').replace(/_/g,' ')} &nbsp;·&nbsp; <b>pos</b> ${pos}<br>
      <b>alt</b> ${h.altitude_m ?? h.terrain_z ?? '—'} m &nbsp;·&nbsp; <b>gps err</b> ${h.gps_error_m ?? '—'} m<br>
      <b>bat</b> ${h.battery_pct ?? '—'}% &nbsp;·&nbsp; <b>weather</b> ${h.weather ?? '—'}<br>
      <b>ctrl</b> ${h.manual_control ? 'manual' : 'auto'} &nbsp;·&nbsp; <b>sos count</b> ${h.sos_count ?? 0}
    </div>`;
  document.getElementById('lnk-ctx').textContent = h.hiker_id;
}

/* ══════════════════════════════════════
   LINKS
══════════════════════════════════════ */
function nearestRelay(hikerId) {
  const h = S.hikers[hikerId], nodes = S.scenario?.lora_nodes || [];
  if (!h || !nodes.length || h.x === undefined) return null;
  let best = null, bd = Infinity;
  for (const n of nodes) { const d = Math.hypot(h.x - n.x, h.y - n.y); if (d < bd) { bd = d; best = n; } }
  if (!best) return null;
  return { hiker_id: hikerId, delivered: true, estimated: true, links: [{ from: hikerId, to: best.name, distance_m: bd, rx_dbm: null, snr_db: null, margin_db: 12, estimated: true }] };
}

function selLinkEv() {
  if (!S.selectedId) return S.events.find(e => Array.isArray(e.links) && e.links.length) || null;
  const a = S.events.find(e => e.hiker_id === S.selectedId && Array.isArray(e.links) && e.links.length);
  return a || nearestRelay(S.selectedId) || S.events.find(e => Array.isArray(e.links) && e.links.length) || null;
}

function renderLinks() {
  const ev = selLinkEv();
  if (!ev || !Array.isArray(ev.links) || !ev.links.length) {
    document.getElementById('lnk-list').innerHTML = '<div class="empty">klik pendaki untuk lihat relasi link</div>';
    ML.links.clearLayers();
    return;
  }

  const lut = {};
  if (S.scenario?.base_station) lut[S.scenario.base_station.name] = S.scenario.base_station;
  (S.scenario?.lora_nodes || []).forEach(n => { lut[n.name] = n; });
  Object.values(S.hikers || {}).forEach(h => { if (h.x !== undefined) lut[h.hiker_id] = h; });

  const marColFn = m => {
    const n = Number(m);
    return Number.isFinite(n) ? (n >= 10 ? getCSS('--green') : n >= 0 ? getCSS('--amber') : getCSS('--red')) : getCSS('--blue');
  };

  document.getElementById('lnk-list').innerHTML = ev.links.map(lk => {
    const mc = marColFn(lk.margin_db);
    const qualLabel = Number.isFinite(Number(lk.margin_db)) ? (Number(lk.margin_db) >= 10 ? 'kuat' : Number(lk.margin_db) >= 0 ? 'marginal' : 'drop') : '—';
    const qualBdg   = Number.isFinite(Number(lk.margin_db)) ? (Number(lk.margin_db) >= 10 ? 'ok' : Number(lk.margin_db) >= 0 ? 'warn' : 'bad') : 'info';
    return `<div class="lnk">
      <div class="ltit">
        <span>${lk.from} → ${lk.to}</span>
        <span class="bdg ${qualBdg}">${qualLabel}</span>
      </div>
      <div class="lmeta">
        route: ${(ev.route && ev.route.length) ? ev.route.join(' → ') : (lk.estimated ? '(estimasi relay)' : '—')}<br>
        jarak: ${Number.isFinite(Number(lk.distance_m)) ? fmt(lk.distance_m, 0) + ' m' : '—'} &nbsp;·&nbsp; margin: <span style="color:${mc}">${fmt(lk.margin_db,1)} dB</span><br>
        RSSI: ${lk.rx_dbm != null ? fmt(lk.rx_dbm,1) + ' dBm' : '—'} &nbsp;·&nbsp; SNR: ${lk.snr_db != null ? fmt(lk.snr_db,1) + ' dB' : '—'}
      </div>
    </div>`;
  }).join('');

  ML.links.clearLayers();
  ev.links.forEach(lk => {
    const fr = lut[lk.from], to = lut[lk.to];
    if (!fr || !to || fr.x === undefined || to.x === undefined) return;
    const col = marColFn(lk.margin_db);
    L.polyline([xy(fr.x, fr.y), xy(to.x, to.y)], {
      color: col, weight: 2.5, opacity: lk.estimated ? .45 : .82,
      dashArray: lk.estimated ? '5 8' : null
    }).bindTooltip(`${lk.from} → ${lk.to}<br>margin ${fmt(lk.margin_db,1)} dB`).addTo(ML.links);
  });
}

/* ══════════════════════════════════════
   EVENTS
══════════════════════════════════════ */
function renderEvents() {
  const rows = (S.events || []).slice(0, 10);
  document.getElementById('ev-cnt').textContent = String(rows.length);
  document.getElementById('ev-list').innerHTML = rows.length ? rows.map(ev => {
    const r = Array.isArray(ev.route) && ev.route.length ? ev.route.join(' → ') : '—';
    const st = ev.delivered ? 'delivered' : (ev.drop_reason || 'drop');
    return `<div class="ev ${ev.delivered?'ok':'drop'}">
      <div class="etit"><span>${ev.hiker_id||'hiker'}</span><span class="bdg ${ev.delivered?'ok':'warn'}">${st}</span></div>
      <div class="emeta">route: ${r}<br>latency: ${fmt(ev.end_to_end_latency_ms,0)} ms &nbsp;·&nbsp; SNR: ${fmt(ev.snr_db,1)} dB</div>
    </div>`;
  }).join('') : '<div class="empty">menunggu event lora…</div>';
}

/* ══════════════════════════════════════
   SOS
══════════════════════════════════════ */
function renderSos() {
  const rows = (S.sosEvents || []).slice(0, 8);
  document.getElementById('sos-cnt').textContent = String(rows.length);
  document.getElementById('sos-list').innerHTML = rows.length ? rows.map(it => `
    <div class="sosev">
      <div class="etit"><span>${it.hiker_id||'hiker'}</span><span class="bdg bad">🆘 sos</span></div>
      <div class="smeta">source: ${it.source||'—'} &nbsp;·&nbsp; t: ${fmt(it.stamp,1)}</div>
    </div>`).join('') : '<div class="empty">belum ada SOS 👍</div>';
}

/* ══════════════════════════════════════
   STATS + HISTORY
══════════════════════════════════════ */
function renderStats() {
  const latest = S.events[0] || null;
  const scopedTrend = activeTrend();
  const overallLat = lastVal(H.overall.lat);
  const pdrV = S.selectedId ? lastVal(scopedTrend.pdr) : (Number.isFinite(Number(S.stats.pdr_pct)) ? Number(S.stats.pdr_pct) : lastVal(scopedTrend.pdr));
  const latV = lastVal(scopedTrend.lat);
  const snrV = lastVal(scopedTrend.snr);
  const mrgV = lastVal(scopedTrend.mrg);

  document.getElementById('mv-pdr').textContent = `${fmt(S.stats.pdr_pct || 0, 1)}%`;
  document.getElementById('mv-lat').textContent = overallLat != null ? `${fmt(overallLat, 0)} ms` : '—';
  document.getElementById('lat-now').textContent = overallLat != null ? `${fmt(overallLat, 0)} ms` : '— ms';
  document.getElementById('lat-cap').textContent = overallLat != null ? `${fmt(overallLat, 0)} ms · target 30 s` : '—';
  document.getElementById('sum-st').textContent  = S.events.length ? 'live' : 'waiting';

  document.getElementById('trend-scope').textContent = S.selectedId ? `${S.selectedId} · 60 sampel terakhir` : 'keseluruhan · 60 sampel terakhir';
  document.getElementById('sp-pdr-v').textContent = pdrV != null ? `${fmt(pdrV, 1)}%` : '—';
  document.getElementById('sp-lat-v').textContent = latV != null ? `${fmt(latV, 0)} ms` : '—';
  document.getElementById('sp-snr-v').textContent = snrV != null ? `${fmt(snrV, 1)} dB` : '—';
  document.getElementById('sp-mrg-v').textContent = mrgV != null ? `${fmt(mrgV, 1)} dB` : '—';

  const sf = latest?.sf ?? S.scenario?.default_sf ?? '—';
  const toa = latest?.toa_ms ?? latest?.time_on_air_ms ?? '—';
  const wk  = (latest?.weather || S.scenario?.weather || 'clear').toString();
  document.getElementById('rf-tag').textContent  = wLabel[wk] || wk;
  document.getElementById('rf-line').textContent = `sf ${sf} / ${toa} ms`;
  document.getElementById('rf-sh').textContent   = latest ? `route: ${latest.route?.[1] || latest.entry_node || '—'}` : 'waiting';
}

/* ══════════════════════════════════════
   TAB SWITCH
══════════════════════════════════════ */
function showTabs() {
  const active = document.querySelector('.tab.active')?.dataset.tab || 'hikers';
  ['hikers','links','events','sos'].forEach(t => {
    document.getElementById('tp-' + t).style.display = t === active ? 'flex' : 'none';
  });
}

/* ══════════════════════════════════════
   CONNECTION STATUS
══════════════════════════════════════ */
function setConn(state, label) {
  const dot = document.getElementById('cdot');
  const lbl = document.getElementById('clbl');
  if (lbl) lbl.textContent = label;
  if (dot) { dot.className = 'conn-dot'; dot.classList.add(state); }
}

/* ══════════════════════════════════════
   RENDER ALL
══════════════════════════════════════ */
function renderAll() {
  if (S.scenario && !S._sceneDone) { renderScene(); S._sceneDone = true; }
  renderHikers();
  renderSelected();
  renderLinks();
  renderEvents();
  renderSos();
  renderStats();
  showTabs();
  redrawSparks();
}

/* ══════════════════════════════════════
   WS HANDLERS
══════════════════════════════════════ */
function onSnapshot(d) {
  S.scenario   = d.scenario || S.scenario;
  S.hikers     = d.hikers   || {};
  S.stats      = d.stats    || S.stats;
  S.events     = Array.isArray(d.events)     ? d.events     : [];
  S.sosEvents  = Array.isArray(d.sos_events) ? d.sos_events : [];
  rebuildTrends(S.events);
  scheduleRender();
}
function onNetworkEv(d) {
  const ev = d.event || d;
  if (!ev || !ev.hiker_id) return;
  S.events = [ev, ...S.events].slice(0, 12);
  if (d.stats) S.stats = d.stats;
  addTrendEvent(H.overall, ev, d.stats?.pdr_pct ?? S.stats?.pdr_pct);
  addTrendEvent(hikerTrend(ev.hiker_id), ev);
  if (ev.end_to_end_latency_ms != null) ekgPush(Number(ev.end_to_end_latency_ms));
  if (!S.hikers[ev.hiker_id]) S.hikers[ev.hiker_id] = { hiker_id: ev.hiker_id };
  if (ev.sos_active !== undefined) S.hikers[ev.hiker_id] = { ...(S.hikers[ev.hiker_id] || { hiker_id: ev.hiker_id }), sos_active: Boolean(ev.sos_active) };
  scheduleRender();
}
function onStatus(d) {
  if (!d || !d.hiker_id) return;
  S.hikers[d.hiker_id] = { ...(S.hikers[d.hiker_id] || { hiker_id: d.hiker_id }), ...d };
  scheduleRender();
}
function onSos(d) {
  if (!d || !d.hiker_id) return;
  const ev = { type:'sos', hiker_id: d.hiker_id, active: d.active !== false, source: d.source || 'unknown', stamp: d.stamp || Date.now()/1000 };
  if (ev.active) S.sosEvents = [ev, ...S.sosEvents].slice(0, 8);
  S.hikers[d.hiker_id] = { ...(S.hikers[d.hiker_id] || { hiker_id: d.hiker_id }), sos_active: ev.active };
  scheduleRender();
}

/* ══════════════════════════════════════
   WEBSOCKET
══════════════════════════════════════ */
let ws=null, retryT=null, openT=null, closedByUs=false, attempt=0;
function wsUrls() {
  const p = location.protocol === 'https:' ? 'wss:' : 'ws:';
  return [...new Set([`${p}//localhost:8080/ws`, `${p}//127.0.0.1:8080/ws`, location.host ? `${p}//${location.host}/ws` : null].filter(Boolean))];
}
function retry(ni) { if (!closedByUs) { attempt++; setConn('wait', attempt===1?'reconnecting…':'retrying…'); clearTimeout(retryT); retryT = setTimeout(() => connect(ni), Math.min(6000, 700 + attempt * 800)); } }
function connect(ni=0) {
  if (closedByUs) return;
  clearTimeout(retryT); clearTimeout(openT);
  const urls = wsUrls(), url = urls[Math.min(ni, urls.length-1)];
  setConn('wait', attempt === 0 ? 'connecting…' : 'reconnecting…');
  try { ws = new WebSocket(url); } catch(_) { retry(ni+1); return; }
  openT = setTimeout(() => { try { if (ws && ws.readyState === WebSocket.CONNECTING) ws.close(); } catch(_){} }, 4500);
  ws.onopen  = () => { clearTimeout(openT); attempt = 0; setConn('live', 'live'); };
  ws.onmessage = m => { try { const d = JSON.parse(m.data); if (d.type==='snapshot')onSnapshot(d); else if(d.type==='status')onStatus(d); else if(d.type==='network_event')onNetworkEv(d); else if(d.type==='sos')onSos(d); } catch(_){} };
  ws.onerror = () => { clearTimeout(openT); setConn('err', 'error'); };
  ws.onclose = () => { clearTimeout(openT); if (!closedByUs) retry(ni+1); };
}

/* ══════════════════════════════════════
   UI BINDINGS
══════════════════════════════════════ */
document.getElementById('lt').onclick = () => document.getElementById('ls').classList.toggle('collapsed');
document.getElementById('rt').onclick = () => document.getElementById('rs').classList.toggle('collapsed');

document.querySelectorAll('.tab').forEach(b => {
  b.addEventListener('click', () => {
    document.querySelectorAll('.tab').forEach(x => x.classList.toggle('active', x === b));
    showTabs();
  });
});

window.addEventListener('resize', redrawSparks);
window.addEventListener('beforeunload', () => { closedByUs=true; clearTimeout(retryT); clearTimeout(openT); try { ws && ws.close(); } catch(_){} });

/* ══════════════════════════════════════
   BOOT
══════════════════════════════════════ */
// permanent delegation — set ONCE, not per-render
document.getElementById('hk-list').addEventListener('mousedown', e => {
  const card = e.target.closest('.hcard');
  if (!card) return;
  toggleSelectedHiker(card.dataset.hid);
});

ekgInit();
ekgStart();
renderAll();
connect();
})();
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


def _terrain_payload(extent: float = WORLD_EXTENT, step: float = 1.0) -> Dict:
    values = []
    min_h = float("inf")
    max_h = float("-inf")

    count = int((extent * 2.0) / step)
    start = -extent + step / 2.0
    for yi in range(count):
        y = start + yi * step
        for xi in range(count):
            x = start + xi * step
            h = terrain_height_world(x, y)
            min_h = min(min_h, h)
            max_h = max(max_h, h)
            values.append(round(h, 3))

    return {
        "step": step,
        "rows": count,
        "cols": count,
        "min_h": round(min_h, 3),
        "max_h": round(max_h, 3),
        "values": values,
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
            "terrain": _terrain_payload(WORLD_EXTENT),
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
