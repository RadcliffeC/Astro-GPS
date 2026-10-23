/**
 * MAX-M10S Rocket Ground Station Web Application
 * Real-time GNSS flight telemetry, live trajectory mapping, dynamics charts, and mission control.
 */

(function () {
    'use strict';

    // Application State
    const state = {
        isMetric: true,
        audioEnabled: false,
        followRocket: true,
        currentLayer: 'dark', // 'dark' | 'satellite'
        telemetry: {},
        history: [],
        events: [],
        pathPoints: [],
        lastPhase: 'PAD',
        wsConnected: false,
        map: null,
        rocketMarker: null,
        padMarker: null,
        apogeeMarker: null,
        flightPathPolyline: null,
        altChart: null,
        velChart: null,
        speechSynth: window.speechSynthesis || null,
        lastSpokenPhase: null
    };

    // Constants & Conversions
    const M_TO_FT = 3.28084;
    const MPS_TO_FPS = 3.28084;
    const MPS_TO_MPH = 2.23694;
    const MPS_TO_KPH = 3.6;

    // DOM Elements
    const elements = {
        flightPhaseBadge: document.getElementById('flight-phase-badge'),
        metClock: document.getElementById('met-clock'),
        utcClock: document.getElementById('utc-clock'),
        signalDot: document.getElementById('signal-dot'),
        signalText: document.getElementById('signal-text'),
        packetRateVal: document.getElementById('packet-rate-val'),
        wsStatus: document.getElementById('ws-status'),
        
        btnSim: document.getElementById('btn-sim'),
        btnSimText: document.getElementById('btn-sim-text'),
        btnSerialModal: document.getElementById('btn-serial-modal'),
        btnPadModal: document.getElementById('btn-pad-modal'),
        btnReset: document.getElementById('btn-reset'),
        btnUnits: document.getElementById('btn-units'),
        unitsLabel: document.getElementById('units-label'),
        btnAudio: document.getElementById('btn-audio'),
        audioIcon: document.getElementById('audio-icon'),
        btnExportDropdown: document.getElementById('btn-export-dropdown'),
        exportMenu: document.getElementById('export-menu'),

        // HUD Elements
        altAglVal: document.getElementById('alt-agl-val'),
        altMslVal: document.getElementById('alt-msl-val'),
        maxAltVal: document.getElementById('max-alt-val'),
        altitudeFill: document.getElementById('altitude-fill'),
        altitudeApogeeMarker: document.getElementById('altitude-apogee-marker'),
        unitAltLabels: document.querySelectorAll('.unit-alt'),

        vspeedVal: document.getElementById('vspeed-val'),
        gspeedVal: document.getElementById('gspeed-val'),
        gspeedAltUnit: document.getElementById('gspeed-alt-unit'),
        totVelVal: document.getElementById('tot-vel-val'),
        maxVelVal: document.getElementById('max-vel-val'),
        machIndicator: document.getElementById('mach-indicator'),
        vviBar: document.getElementById('vvi-bar'),
        unitSpeedLabels: document.querySelectorAll('.unit-speed'),

        headingVal: document.getElementById('heading-val'),
        compassNeedle: document.getElementById('compass-needle'),
        distPadVal: document.getElementById('dist-pad-val'),
        slantRangeVal: document.getElementById('slant-range-val'),
        azimuthVal: document.getElementById('azimuth-val'),
        elevationVal: document.getElementById('elevation-val'),
        unitDistLabels: document.querySelectorAll('.unit-dist'),

        fixBadge: document.getElementById('fix-badge'),
        latVal: document.getElementById('lat-val'),
        lonVal: document.getElementById('lon-val'),
        satsVal: document.getElementById('sats-val'),
        satBars: document.getElementById('sat-bars'),
        hdopVal: document.getElementById('hdop-val'),
        vdopVal: document.getElementById('vdop-val'),
        padSummary: document.getElementById('pad-summary'),
        btnCopyCoords: document.getElementById('btn-copy-coords'),
        btnMapsLink: document.getElementById('btn-maps-link'),

        // Map controls
        btnMapCenter: document.getElementById('btn-map-center'),
        btnMapLayer: document.getElementById('btn-map-layer'),
        btnClearPath: document.getElementById('btn-clear-path'),
        mapStatusVal: document.getElementById('map-status-val'),
        trackPointsCount: document.getElementById('track-points-count'),
        mapSlantVal: document.getElementById('map-slant-val'),

        // Logs and Tabs
        tabBtnEvents: document.getElementById('tab-btn-events'),
        tabBtnNmea: document.getElementById('tab-btn-nmea'),
        tabEvents: document.getElementById('tab-events'),
        tabNmea: document.getElementById('tab-nmea'),
        eventsCount: document.getElementById('events-count'),
        eventsList: document.getElementById('events-list'),
        nmeaTerminal: document.getElementById('nmea-terminal'),
        autoscrollToggle: document.getElementById('autoscroll-toggle'),
        btnClearLogs: document.getElementById('btn-clear-logs'),

        // Serial Modal
        modalSerial: document.getElementById('modal-serial'),
        serialPortSelect: document.getElementById('serial-port-select'),
        serialBaudSelect: document.getElementById('serial-baud-select'),
        btnRefreshPorts: document.getElementById('btn-refresh-ports'),
        btnSerialConnect: document.getElementById('btn-serial-connect'),
        btnSerialDisconnect: document.getElementById('btn-serial-disconnect'),
        serialStatusMsg: document.getElementById('serial-status-msg'),

        // Pad Modal
        modalPad: document.getElementById('modal-pad'),
        padLatInput: document.getElementById('pad-lat-input'),
        padLonInput: document.getElementById('pad-lon-input'),
        padAltInput: document.getElementById('pad-alt-input'),
        btnPadLockCurrent: document.getElementById('btn-pad-lock-current'),
        btnPadSave: document.getElementById('btn-pad-save')
    };

    // =========================================================================
    // INITIALIZATION
    // =========================================================================

    function init() {
        initMap();
        initCharts();
        setupEventListeners();
        connectWebSocket();
        fetchInitialData();
    }

    // =========================================================================
    // MAP INITIALIZATION (Leaflet)
    // =========================================================================

    let tileLayers = {};

    function initMap() {
        const defaultLat = 32.9904;
        const defaultLon = -106.9750;

        state.map = L.map('map-container', {
            center: [defaultLat, defaultLon],
            zoom: 14,
            zoomControl: true,
            attributionControl: false
        });

        // Dark Matter Base Layer
        tileLayers.dark = L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', {
            maxZoom: 19,
            subdomains: 'abcd'
        }).addTo(state.map);

        // Satellite Tile Layer (Esri World Imagery)
        tileLayers.satellite = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
            maxZoom: 18
        });

        // Launch Pad Icon
        const padSvg = `
            <svg viewBox="0 0 32 32" width="28" height="28" fill="#ffb700">
                <circle cx="16" cy="16" r="14" stroke="#ffb700" stroke-width="2" fill="rgba(255,183,0,0.2)"/>
                <circle cx="16" cy="16" r="4" fill="#ffffff"/>
                <line x1="16" y1="2" x2="16" y2="30" stroke="#ffb700" stroke-width="1.5"/>
                <line x1="2" y1="16" x2="30" y2="16" stroke="#ffb700" stroke-width="1.5"/>
            </svg>
        `;
        const padIcon = L.divIcon({
            html: padSvg,
            className: 'custom-pad-icon',
            iconSize: [28, 28],
            iconAnchor: [14, 14]
        });

        state.padMarker = L.marker([defaultLat, defaultLon], { icon: padIcon })
            .addTo(state.map)
            .bindPopup('<b>Launch Pad Reference</b><br>Ground Station Zero');

        // Rocket Marker
        const rocketSvg = `
            <div id="map-rocket-rotator" style="transform: rotate(0deg); transform-origin: center;">
                <svg viewBox="0 0 28 28" width="28" height="28">
                    <circle cx="14" cy="14" r="12" fill="rgba(0, 240, 255, 0.2)" stroke="#00f0ff" stroke-width="1.5"/>
                    <polygon points="14,3 20,22 14,18 8,22" fill="#00f0ff" stroke="#ffffff" stroke-width="0.8"/>
                </svg>
            </div>
        `;
        const rocketIcon = L.divIcon({
            html: rocketSvg,
            className: 'custom-rocket-icon',
            iconSize: [28, 28],
            iconAnchor: [14, 14]
        });

        state.rocketMarker = L.marker([defaultLat, defaultLon], { icon: rocketIcon, zIndexOffset: 1000 })
            .addTo(state.map)
            .bindTooltip('Rocket Position', { permanent: false, direction: 'top' });

        // Trajectory Polyline
        state.flightPathPolyline = L.polyline([], {
            color: '#00f0ff',
            weight: 3.5,
            opacity: 0.85,
            smoothFactor: 1.0
        }).addTo(state.map);
    }

    function updateMap(data) {
        if (!state.map || !data.latitude || !data.longitude || (data.latitude === 0 && data.longitude === 0)) {
            return;
        }

        const latLng = [data.latitude, data.longitude];

        // Update Rocket Marker
        state.rocketMarker.setLatLng(latLng);
        const rotator = document.getElementById('map-rocket-rotator');
        if (rotator && data.heading !== undefined) {
            rotator.style.transform = `rotate(${data.heading}deg)`;
        }

        const altFormatted = formatAlt(data.altitude_agl);
        state.rocketMarker.setTooltipContent(`<b>Alt AGL:</b> ${altFormatted}<br><b>V-Speed:</b> ${formatSpeed(data.vertical_speed)}`);

        // Update Pad Marker if changed
        if (data.pad_lat && data.pad_lon) {
            state.padMarker.setLatLng([data.pad_lat, data.pad_lon]);
        }

        // Add to Path
        state.pathPoints.push(latLng);
        state.flightPathPolyline.setLatLngs(state.pathPoints);

        // Update stats
        elements.trackPointsCount.textContent = `${state.pathPoints.length} pts`;
        elements.mapSlantVal.textContent = formatDist(data.pad_slant_range || 0);

        // Auto Pan
        if (state.followRocket) {
            state.map.panTo(latLng, { animate: true, duration: 0.5 });
        }
    }

    // =========================================================================
    // CHARTS (Chart.js)
    // =========================================================================

    function initCharts() {
        const commonOptions = {
            responsive: true,
            maintainAspectRatio: false,
            animation: false,
            parsing: false,
            scales: {
                x: {
                    type: 'linear',
                    grid: { color: 'rgba(255, 255, 255, 0.05)' },
                    ticks: {
                        color: '#64748b',
                        font: { family: 'JetBrains Mono', size: 9 },
                        callback: function (val) { return 'T+' + val.toFixed(0) + 's'; }
                    }
                },
                y: {
                    grid: { color: 'rgba(255, 255, 255, 0.05)' },
                    ticks: {
                        color: '#64748b',
                        font: { family: 'JetBrains Mono', size: 9 }
                    }
                }
            },
            plugins: {
                legend: { display: false },
                tooltip: {
                    enabled: true,
                    backgroundColor: 'rgba(11, 18, 32, 0.9)',
                    titleFont: { family: 'JetBrains Mono', size: 10 },
                    bodyFont: { family: 'JetBrains Mono', size: 10 },
                    borderColor: '#2b3e68',
                    borderWidth: 1
                }
            }
        };

        // Altitude Chart
        const altCtx = document.getElementById('altChart').getContext('2d');
        state.altChart = new Chart(altCtx, {
            type: 'line',
            data: {
                datasets: [{
                    label: 'AGL Altitude',
                    data: [],
                    borderColor: '#00f0ff',
                    borderWidth: 2,
                    pointRadius: 0,
                    tension: 0.1,
                    fill: {
                        target: 'origin',
                        above: 'rgba(0, 240, 255, 0.08)'
                    }
                }]
            },
            options: {
                ...commonOptions,
                scales: {
                    ...commonOptions.scales,
                    y: {
                        ...commonOptions.scales.y,
                        beginAtZero: true,
                        title: { display: true, text: 'Altitude (m)', color: '#64748b', font: { size: 9 } }
                    }
                }
            }
        });

        // Velocity Chart
        const velCtx = document.getElementById('velChart').getContext('2d');
        state.velChart = new Chart(velCtx, {
            type: 'line',
            data: {
                datasets: [
                    {
                        label: 'Vertical Speed',
                        data: [],
                        borderColor: '#00ff88',
                        borderWidth: 1.8,
                        pointRadius: 0,
                        tension: 0.1
                    },
                    {
                        label: 'Ground Speed',
                        data: [],
                        borderColor: '#ffb700',
                        borderWidth: 1.5,
                        borderDash: [3, 3],
                        pointRadius: 0,
                        tension: 0.1
                    }
                ]
            },
            options: {
                ...commonOptions,
                scales: {
                    ...commonOptions.scales,
                    y: {
                        ...commonOptions.scales.y,
                        title: { display: true, text: 'Speed (m/s)', color: '#64748b', font: { size: 9 } }
                    }
                }
            }
        });
    }

    function updateCharts(data) {
        if (!state.altChart || !state.velChart || data.met_seconds === undefined) return;

        const met = data.met_seconds;
        const alt = state.isMetric ? data.altitude_agl : data.altitude_agl * M_TO_FT;
        const vspeed = state.isMetric ? data.vertical_speed : data.vertical_speed * MPS_TO_FPS;
        const gspeed = state.isMetric ? data.ground_speed : data.ground_speed * MPS_TO_FPS;

        // Push points
        state.altChart.data.datasets[0].data.push({ x: met, y: alt });
        state.velChart.data.datasets[0].data.push({ x: met, y: vspeed });
        state.velChart.data.datasets[1].data.push({ x: met, y: gspeed });

        // Keep maximum 300 data points in charts for fast render
        if (state.altChart.data.datasets[0].data.length > 300) {
            state.altChart.data.datasets[0].data.shift();
            state.velChart.data.datasets[0].data.shift();
            state.velChart.data.datasets[1].data.shift();
        }

        state.altChart.update('none');
        state.velChart.update('none');
    }

    // =========================================================================
    // WEBSOCKET & TELEMETRY STREAM
    // =========================================================================

    function connectWebSocket() {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${protocol}//${window.location.host}/ws/telemetry`;

        elements.wsStatus.textContent = 'WS CONNECTING...';
        elements.wsStatus.style.color = 'var(--accent-amber)';

        const socket = new WebSocket(wsUrl);

        socket.onopen = function () {
            state.wsConnected = true;
            elements.wsStatus.textContent = 'WS LIVE';
            elements.wsStatus.style.color = 'var(--accent-emerald)';
        };

        socket.onmessage = function (event) {
            try {
                const message = JSON.parse(event.data);
                handleWebSocketMessage(message);
            } catch (err) {
                console.error('Error parsing telemetry frame:', err);
            }
        };

        socket.onclose = function () {
            state.wsConnected = false;
            elements.wsStatus.textContent = 'WS RECONNECTING';
            elements.wsStatus.style.color = 'var(--accent-red)';
            setTimeout(connectWebSocket, 2000);
        };

        socket.onerror = function () {
            socket.close();
        };
    }

    function handleWebSocketMessage(msg) {
        if (msg.type === 'INITIAL_SYNC') {
            if (msg.state) renderTelemetry(msg.state);
            if (msg.events) {
                elements.eventsList.innerHTML = '';
                msg.events.forEach(addEventToUI);
            }
            if (msg.raw_nmea) {
                elements.nmeaTerminal.innerHTML = '';
                msg.raw_nmea.forEach(addNmeaToUI);
            }
            if (msg.history && msg.history.length > 0) {
                state.pathPoints = [];
                msg.history.forEach(h => {
                    if (h.lat && h.lon) state.pathPoints.push([h.lat, h.lon]);
                });
                state.flightPathPolyline.setLatLngs(state.pathPoints);
            }
        } else if (msg.type === 'TELEMETRY_UPDATE' || msg.type === 'HEARTBEAT') {
            if (msg.state) {
                renderTelemetry(msg.state);
            }
        }
    }

    // =========================================================================
    // TELEMETRY HUD RENDERING
    // =========================================================================

    function renderTelemetry(data) {
        state.telemetry = data;

        // Header Timers
        if (data.met_formatted) elements.metClock.textContent = data.met_formatted;
        if (data.timestamp_utc) elements.utcClock.textContent = data.timestamp_utc + ' UTC';

        // Signal & Rate
        elements.packetRateVal.textContent = (data.packet_rate || 0).toFixed(1);
        updateSignalStatus(data);

        // Flight Phase
        updateFlightPhase(data.flight_phase);

        // Simulation Button State
        if (data.simulating) {
            elements.btnSim.classList.remove('btn-primary');
            elements.btnSim.classList.add('btn-outline-danger');
            elements.btnSimText.textContent = 'Stop Sim';
        } else {
            elements.btnSim.classList.remove('btn-outline-danger');
            elements.btnSim.classList.add('btn-primary');
            elements.btnSimText.textContent = 'Start Sim';
        }

        // Altitude Metrics
        const altAgl = data.altitude_agl || 0;
        const altMsl = data.altitude_msl || 0;
        const maxAltAgl = data.max_altitude_agl || 0;

        elements.altAglVal.textContent = state.isMetric ? altAgl.toFixed(1) : (altAgl * M_TO_FT).toFixed(1);
        elements.altMslVal.textContent = state.isMetric ? altMsl.toFixed(1) : (altMsl * M_TO_FT).toFixed(1);
        elements.maxAltVal.textContent = state.isMetric ? maxAltAgl.toFixed(1) : (maxAltAgl * M_TO_FT).toFixed(1);

        // Altitude Visual Bar (0 to max recorded or 5000m)
        const scaleMax = Math.max(5000, maxAltAgl * 1.1);
        const fillPercent = Math.min(100, Math.max(0, (altAgl / scaleMax) * 100));
        elements.altitudeFill.style.width = fillPercent + '%';
        const apogeePercent = Math.min(100, Math.max(0, (maxAltAgl / scaleMax) * 100));
        elements.altitudeApogeeMarker.style.left = apogeePercent + '%';

        // Velocity & Kinematics
        const vspeed = data.vertical_speed || 0;
        const gspeed = data.ground_speed || 0;
        const totVel = data.total_velocity || 0;
        const maxVel = data.max_velocity || 0;

        const vspeedStr = (vspeed >= 0 ? '+' : '') + (state.isMetric ? vspeed.toFixed(1) : (vspeed * MPS_TO_FPS).toFixed(1));
        elements.vspeedVal.textContent = vspeedStr;
        elements.vspeedVal.className = vspeed >= 0 ? 'vspeed-positive' : 'vspeed-negative';

        elements.gspeedVal.textContent = state.isMetric ? gspeed.toFixed(1) : (gspeed * MPS_TO_FPS).toFixed(1);
        elements.gspeedAltUnit.textContent = state.isMetric 
            ? `${(gspeed * MPS_TO_KPH).toFixed(1)} km/h` 
            : `${(gspeed * MPS_TO_MPH).toFixed(1)} mph`;

        elements.totVelVal.textContent = state.isMetric ? totVel.toFixed(1) : (totVel * MPS_TO_FPS).toFixed(1);
        elements.maxVelVal.textContent = state.isMetric ? maxVel.toFixed(1) : (maxVel * MPS_TO_FPS).toFixed(1);

        // Mach Number (approx 340 m/s speed of sound)
        const mach = (totVel / 340.0).toFixed(2);
        elements.machIndicator.textContent = `Mach ${mach}`;

        // VVI Meter (bidirectional ±50 m/s range)
        const vviPercent = Math.min(50, Math.max(-50, (vspeed / 100.0) * 50));
        if (vviPercent >= 0) {
            elements.vviBar.style.left = '50%';
            elements.vviBar.style.width = vviPercent + '%';
            elements.vviBar.style.background = 'var(--accent-emerald)';
        } else {
            elements.vviBar.style.left = (50 + vviPercent) + '%';
            elements.vviBar.style.width = Math.abs(vviPercent) + '%';
            elements.vviBar.style.background = 'var(--accent-red)';
        }

        // Compass & Ground Vector
        const heading = (data.heading || 0).toFixed(0);
        elements.headingVal.textContent = heading.padStart(3, '0');
        elements.compassNeedle.style.transform = `rotate(${heading}deg)`;

        elements.distPadVal.textContent = formatDist(data.pad_distance || 0);
        elements.slantRangeVal.textContent = formatDist(data.pad_slant_range || 0);
        elements.azimuthVal.textContent = (data.pad_bearing || 0).toFixed(1);
        elements.elevationVal.textContent = (data.pad_elevation || 0).toFixed(1);

        // GPS Coordinates & Status
        const lat = data.latitude || 0;
        const lon = data.longitude || 0;
        const latDir = lat >= 0 ? 'N' : 'S';
        const lonDir = lon >= 0 ? 'E' : 'W';
        elements.latVal.textContent = `${Math.abs(lat).toFixed(6)}° ${latDir}`;
        elements.lonVal.textContent = `${Math.abs(lon).toFixed(6)}° ${lonDir}`;
        elements.btnMapsLink.href = `https://www.google.com/maps?q=${lat},${lon}`;

        // Fix Quality Badge
        elements.fixBadge.textContent = data.fix_quality || 'NO FIX';
        if (data.has_fix) {
            elements.fixBadge.className = 'badge badge-fix-3d';
        } else {
            elements.fixBadge.className = 'badge badge-fix-no';
        }

        // Satellites & Signal Bars
        const sats = data.satellites || 0;
        elements.satsVal.textContent = sats;
        elements.satBars.className = 'sat-signal-bars';
        if (sats >= 10) elements.satBars.classList.add('sat-bars-active-4');
        else if (sats >= 6) elements.satBars.classList.add('sat-bars-active-3');
        else if (sats >= 4) elements.satBars.classList.add('sat-bars-active-2');
        else if (sats > 0) elements.satBars.classList.add('sat-bars-active-1');

        elements.hdopVal.textContent = data.hdop ? data.hdop.toFixed(1) : '--';
        elements.vdopVal.textContent = data.vdop ? data.vdop.toFixed(1) : '--';

        if (data.pad_lat && data.pad_lon) {
            elements.padSummary.textContent = `${data.pad_lat.toFixed(4)}, ${data.pad_lon.toFixed(4)}`;
        }

        // Map and Charts Update
        updateMap(data);
        updateCharts(data);
    }

    function updateSignalStatus(data) {
        const dot = elements.signalDot;
        const txt = elements.signalText;

        dot.className = 'dot';
        if (data.signal_status === 'LIVE') {
            dot.classList.add('dot-live');
            txt.textContent = 'LIVE FEED';
            txt.style.color = 'var(--accent-emerald)';
        } else if (data.signal_status === 'SIMULATION') {
            dot.classList.add('dot-sim');
            txt.textContent = 'SIMULATION';
            txt.style.color = 'var(--accent-amber)';
        } else if (data.signal_status === 'LOS') {
            dot.classList.add('dot-los');
            txt.textContent = 'SIGNAL LOST';
            txt.style.color = 'var(--accent-red)';
        } else {
            dot.classList.add('dot-disconnected');
            txt.textContent = 'NO CARRIER';
            txt.style.color = '#64748b';
        }
    }

    function updateFlightPhase(phase) {
        if (!phase) phase = 'PAD';
        const badge = elements.flightPhaseBadge;
        badge.textContent = phase;
        badge.className = 'phase-badge phase-' + phase.toLowerCase();

        if (phase !== state.lastPhase) {
            state.lastPhase = phase;
            speakPhaseChange(phase);
        }
    }

    // =========================================================================
    // VOICE & AUDIO ALERTS
    // =========================================================================

    function speakPhaseChange(phase) {
        if (!state.audioEnabled || !state.speechSynth) return;
        if (state.lastSpokenPhase === phase) return;

        let speechText = '';
        switch (phase) {
            case 'BOOST': speechText = 'Rocket ignition and launch detected.'; break;
            case 'COAST': speechText = 'Motor burnout, entering coast phase.'; break;
            case 'APOGEE': speechText = `Apogee reached at ${formatAlt(state.telemetry.altitude_agl || 0)}.`; break;
            case 'DROGUE': speechText = 'Drogue parachute deployed.'; break;
            case 'MAIN': speechText = 'Main recovery parachute deployed.'; break;
            case 'LANDED': speechText = 'Rocket touchdown confirmed.'; break;
            default: break;
        }

        if (speechText) {
            state.lastSpokenPhase = phase;
            const utter = new SpeechSynthesisUtterance(speechText);
            utter.rate = 1.05;
            utter.pitch = 1.0;
            state.speechSynth.speak(utter);
        }
    }

    // =========================================================================
    // EVENTS & NMEA LOGGING
    // =========================================================================

    function addEventToUI(evt) {
        const item = document.createElement('div');
        item.className = 'event-item';
        if (evt.type === 'LAUNCH') item.classList.add('event-launch');
        else if (evt.type === 'PHASE_CHANGE' && evt.data && evt.data.new === 'APOGEE') item.classList.add('event-apogee');
        else if (evt.type === 'PHASE_CHANGE' && evt.data && evt.data.new === 'LANDED') item.classList.add('event-landed');

        item.innerHTML = `
            <span class="event-time">${evt.met || evt.timestamp}</span>
            <span class="event-badge badge-info">${evt.type}</span>
            <span class="event-msg">${evt.description}</span>
        `;
        elements.eventsList.appendChild(item);

        elements.eventsCount.textContent = elements.eventsList.children.length;

        if (elements.autoscrollToggle.checked) {
            elements.tabEvents.scrollTop = elements.tabEvents.scrollHeight;
        }
    }

    function addNmeaToUI(entry) {
        const line = document.createElement('div');
        line.className = 'term-line';

        const raw = entry.raw || '';
        if (raw.includes('GGA')) line.classList.add('term-gga');
        else if (raw.includes('RMC')) line.classList.add('term-rmc');
        else if (raw.includes('VTG')) line.classList.add('term-vtg');
        else if (raw.includes('GSA')) line.classList.add('term-gsa');

        line.textContent = `[${entry.time}] ${raw}`;
        elements.nmeaTerminal.appendChild(line);

        // Keep maximum 80 lines in terminal
        if (elements.nmeaTerminal.children.length > 80) {
            elements.nmeaTerminal.removeChild(elements.nmeaTerminal.firstChild);
        }

        if (elements.autoscrollToggle.checked) {
            elements.tabNmea.scrollTop = elements.tabNmea.scrollHeight;
        }
    }

    // =========================================================================
    // EVENT LISTENERS & MODAL HANDLERS
    // =========================================================================

    function setupEventListeners() {
        // Simulator Toggle Button
        elements.btnSim.addEventListener('click', async () => {
            const isSimulating = state.telemetry.simulating;
            const endpoint = isSimulating ? '/api/simulation/stop' : '/api/simulation/start';
            try {
                await fetch(endpoint, { method: 'POST' });
            } catch (e) {
                console.error('Error toggling sim:', e);
            }
        });

        // Units Toggle Button
        elements.btnUnits.addEventListener('click', () => {
            state.isMetric = !state.isMetric;
            elements.unitsLabel.textContent = state.isMetric ? 'METRIC' : 'IMPERIAL';
            elements.unitAltLabels.forEach(el => el.textContent = state.isMetric ? 'm' : 'ft');
            elements.unitSpeedLabels.forEach(el => el.textContent = state.isMetric ? 'm/s' : 'ft/s');
            elements.unitDistLabels.forEach(el => el.textContent = state.isMetric ? 'm' : 'ft');
            if (state.telemetry) renderTelemetry(state.telemetry);
        });

        // Audio Alert Toggle
        elements.btnAudio.addEventListener('click', () => {
            state.audioEnabled = !state.audioEnabled;
            elements.audioIcon.innerHTML = state.audioEnabled ? '&#128264;' : '&#128266;';
            elements.btnAudio.style.borderColor = state.audioEnabled ? 'var(--accent-emerald)' : '';
            if (state.audioEnabled && state.speechSynth) {
                const testUtter = new SpeechSynthesisUtterance('Audio telemetry active.');
                state.speechSynth.speak(testUtter);
            }
        });

        // Reset Button
        elements.btnReset.addEventListener('click', async () => {
            if (confirm('Reset mission metrics, flight path, and apogee tracker?')) {
                await fetch('/api/reset', { method: 'POST' });
                state.pathPoints = [];
                state.flightPathPolyline.setLatLngs([]);
                state.altChart.data.datasets[0].data = [];
                state.velChart.data.datasets[0].data = [];
                state.velChart.data.datasets[1].data = [];
                state.altChart.update();
                state.velChart.update();
            }
        });

        // Export Dropdown
        elements.btnExportDropdown.addEventListener('click', (e) => {
            e.stopPropagation();
            elements.btnExportDropdown.parentElement.classList.toggle('open');
        });
        document.addEventListener('click', () => {
            document.querySelectorAll('.dropdown.open').forEach(d => d.classList.remove('open'));
        });

        // Map Follow Rocket Toggle
        elements.btnMapCenter.addEventListener('click', () => {
            state.followRocket = !state.followRocket;
            elements.btnMapCenter.classList.toggle('active', state.followRocket);
            if (state.followRocket && state.telemetry.latitude) {
                state.map.panTo([state.telemetry.latitude, state.telemetry.longitude]);
            }
        });

        // Map Layer Toggle
        elements.btnMapLayer.addEventListener('click', () => {
            if (state.currentLayer === 'dark') {
                state.map.removeLayer(tileLayers.dark);
                tileLayers.satellite.addTo(state.map);
                state.currentLayer = 'satellite';
                elements.btnMapLayer.classList.add('active');
            } else {
                state.map.removeLayer(tileLayers.satellite);
                tileLayers.dark.addTo(state.map);
                state.currentLayer = 'dark';
                elements.btnMapLayer.classList.remove('active');
            }
        });

        // Clear Path
        elements.btnClearPath.addEventListener('click', () => {
            state.pathPoints = [];
            state.flightPathPolyline.setLatLngs([]);
            elements.trackPointsCount.textContent = '0 pts';
        });

        // Tabs
        elements.tabBtnEvents.addEventListener('click', () => {
            elements.tabBtnEvents.classList.add('active');
            elements.tabBtnNmea.classList.remove('active');
            elements.tabEvents.classList.add('active');
            elements.tabNmea.classList.remove('active');
        });
        elements.tabBtnNmea.addEventListener('click', () => {
            elements.tabBtnNmea.classList.add('active');
            elements.tabBtnEvents.classList.remove('active');
            elements.tabNmea.classList.add('active');
            elements.tabEvents.classList.remove('active');
        });

        // Clear Logs
        elements.btnClearLogs.addEventListener('click', () => {
            elements.eventsList.innerHTML = '';
            elements.nmeaTerminal.innerHTML = '';
            elements.eventsCount.textContent = '0';
        });

        // Copy Coordinates
        elements.btnCopyCoords.addEventListener('click', () => {
            const lat = state.telemetry.latitude || 0;
            const lon = state.telemetry.longitude || 0;
            navigator.clipboard.writeText(`${lat.toFixed(6)}, ${lon.toFixed(6)}`);
            elements.btnCopyCoords.textContent = 'Copied!';
            setTimeout(() => elements.btnCopyCoords.textContent = 'Copy', 1500);
        });

        // Modal Open / Close
        elements.btnSerialModal.addEventListener('click', () => {
            openModal(elements.modalSerial);
            refreshSerialPorts();
        });
        elements.btnPadModal.addEventListener('click', () => {
            openModal(elements.modalPad);
            if (state.telemetry.pad_lat) elements.padLatInput.value = state.telemetry.pad_lat.toFixed(6);
            if (state.telemetry.pad_lon) elements.padLonInput.value = state.telemetry.pad_lon.toFixed(6);
            if (state.telemetry.pad_alt) elements.padAltInput.value = state.telemetry.pad_alt.toFixed(1);
        });

        document.querySelectorAll('.modal-close, .modal-close-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const targetId = btn.getAttribute('data-close');
                closeModal(document.getElementById(targetId));
            });
        });

        // Serial Port Config Actions
        elements.btnRefreshPorts.addEventListener('click', refreshSerialPorts);
        elements.btnSerialConnect.addEventListener('click', async () => {
            const port = elements.serialPortSelect.value;
            const baud = parseInt(elements.serialBaudSelect.value, 10);
            try {
                const res = await fetch('/api/connect', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ port: port, baud_rate: baud })
                });
                const result = await res.json();
                if (res.ok) {
                    showStatusMsg(elements.serialStatusMsg, `Connected to ${port} @ ${baud} baud`, 'success');
                    setTimeout(() => closeModal(elements.modalSerial), 1200);
                } else {
                    showStatusMsg(elements.serialStatusMsg, result.detail || 'Connection failed', 'error');
                }
            } catch (err) {
                showStatusMsg(elements.serialStatusMsg, 'Network error connecting to serial port', 'error');
            }
        });

        elements.btnSerialDisconnect.addEventListener('click', async () => {
            await fetch('/api/disconnect', { method: 'POST' });
            showStatusMsg(elements.serialStatusMsg, 'Disconnected', 'success');
        });

        // Launch Pad Save Actions
        elements.btnPadSave.addEventListener('click', async () => {
            const lat = parseFloat(elements.padLatInput.value);
            const lon = parseFloat(elements.padLonInput.value);
            const alt = parseFloat(elements.padAltInput.value);
            await fetch('/api/pad/set', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ latitude: lat, longitude: lon, altitude: alt })
            });
            closeModal(elements.modalPad);
        });

        elements.btnPadLockCurrent.addEventListener('click', async () => {
            try {
                const res = await fetch('/api/pad/lock_current', { method: 'POST' });
                const result = await res.json();
                if (res.ok && result.pad) {
                    elements.padLatInput.value = result.pad.lat.toFixed(6);
                    elements.padLonInput.value = result.pad.lon.toFixed(6);
                    elements.padAltInput.value = result.pad.alt.toFixed(1);
                    closeModal(elements.modalPad);
                } else {
                    alert(result.detail || 'Could not lock pad: No 3D GPS fix');
                }
            } catch (err) {
                alert('Error locking current position');
            }
        });
    }

    function openModal(modalEl) {
        modalEl.classList.remove('hidden');
    }

    function closeModal(modalEl) {
        modalEl.classList.add('hidden');
    }

    function showStatusMsg(el, msg, type) {
        el.textContent = msg;
        el.className = 'status-msg ' + type;
    }

    async function refreshSerialPorts() {
        try {
            const res = await fetch('/api/ports');
            const ports = await res.json();
            elements.serialPortSelect.innerHTML = '';
            if (ports && ports.length > 0) {
                ports.forEach(p => {
                    const opt = document.createElement('option');
                    opt.value = p.device;
                    opt.textContent = `${p.device} (${p.description || 'GPS Device'})`;
                    elements.serialPortSelect.appendChild(opt);
                });
            } else {
                const defaults = ['/dev/ttyUSB0', '/dev/ttyACM0', '/dev/tty.usbserial', 'COM3'];
                defaults.forEach(d => {
                    const opt = document.createElement('option');
                    opt.value = d;
                    opt.textContent = d;
                    elements.serialPortSelect.appendChild(opt);
                });
            }
        } catch (e) {
            console.error('Failed to fetch ports:', e);
        }
    }

    async function fetchInitialData() {
        try {
            const [telRes, histRes, evtRes] = await Promise.all([
                fetch('/api/telemetry'),
                fetch('/api/history'),
                fetch('/api/events')
            ]);
            const tel = await telRes.json();
            const hist = await histRes.json();
            const evts = await evtRes.json();

            if (tel) renderTelemetry(tel);
            if (evts && Array.isArray(evts)) {
                elements.eventsList.innerHTML = '';
                evts.forEach(addEventToUI);
            }
        } catch (e) {
            console.error('Error fetching initial data:', e);
        }
    }

    // =========================================================================
    // FORMATTING HELPERS
    // =========================================================================

    function formatAlt(meters) {
        if (state.isMetric) {
            return `${meters.toFixed(1)} m`;
        } else {
            return `${(meters * M_TO_FT).toFixed(1)} ft`;
        }
    }

    function formatSpeed(mps) {
        if (state.isMetric) {
            return `${mps.toFixed(1)} m/s`;
        } else {
            return `${(mps * MPS_TO_FPS).toFixed(1)} ft/s`;
        }
    }

    function formatDist(meters) {
        if (state.isMetric) {
            if (meters >= 1000) return `${(meters / 1000).toFixed(2)} km`;
            return `${meters.toFixed(0)} m`;
        } else {
            const feet = meters * M_TO_FT;
            if (feet >= 5280) return `${(feet / 5280).toFixed(2)} mi`;
            return `${feet.toFixed(0)} ft`;
        }
    }

    // Run when DOM is ready
    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', init);
    } else {
        init();
    }
})();
