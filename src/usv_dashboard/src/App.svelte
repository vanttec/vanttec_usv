<script>
  import { onDestroy } from 'svelte'
  import { connect, subscribe, connected, rosUrl } from './lib/ros.js'
  import Indicator from './lib/Indicator.svelte'
  import CameraPanel from './lib/CameraPanel.svelte'

  // ── connection ────────────────────────────────────────────────
  let urlInput = 'ws://localhost:9090'
  let ros = null
  let topics = []

  function handleConnect() {
    ros = connect(urlInput)
    rosUrl.set(urlInput)
    topics.forEach(t => t.unsubscribe())
    topics = []
    setupSubscriptions()
  }

  // ── state ─────────────────────────────────────────────────────
  let status = {
    ekf_status: 0,
    gps_pos_status: 0,
    gps_hdt_status: 0,
    can_stm_status: false,
    camera_status: false,
    lidar_status: false,
    op_mode: 2,
    mission_id: 0,
    mission_state: 0,
    mission_status: 0,
    obj_list: []
  }

  let pose = { x: 0, y: 0, theta: 0 }

  // ── labels ────────────────────────────────────────────────────
  const EKF_LABELS = ['UNINITIALIZED', 'VERTICAL_GYRO', 'AHRS', 'NAV_VELOCITY', 'NAV_POSITION']
  const GPS_LABELS = ['NO_SOLUTION', 'UNKNOWN', 'SINGLE', 'PSRDIFF', 'SBAS',
                      'OMNISTAR', 'RTK_FLOAT', 'RTK_INT', 'PPP_FLOAT', 'PPP_INT', 'FIXED']
  const OP_LABELS  = ['AUTO', 'TELEOP', 'INACTIVE']
  const COLOR_NAMES = ['red', 'green', 'blue', 'yellow', 'black']
  const COLOR_HEX   = ['#ef4444', '#22c55e', '#3b82f6', '#eab308', '#6b7280']

  $: ekfLabel     = EKF_LABELS[status.ekf_status]  ?? status.ekf_status
  $: gpsLabel     = GPS_LABELS[status.gps_pos_status] ?? status.gps_pos_status
  $: opLabel      = OP_LABELS[status.op_mode] ?? 'UNKNOWN'
  $: ekfQuality   = status.ekf_status >= 4 ? 'good' : status.ekf_status >= 2 ? 'warn' : 'bad'
  $: gpsQuality   = status.gps_pos_status >= 6 ? 'good' : status.gps_pos_status >= 2 ? 'warn' : 'bad'

  // ── subscriptions ─────────────────────────────────────────────
  function setupSubscriptions() {
    topics.push(subscribe(ros, '/usv/status', 'usv_interfaces/msg/SystemStatus', msg => {
      status = msg
    }))
    topics.push(subscribe(ros, '/usv/state/pose', 'geometry_msgs/msg/Pose2D', msg => {
      pose = msg
    }))
  }

  onDestroy(() => topics.forEach(t => t.unsubscribe()))
</script>

<main>
  <!-- ── top bar ────────────────────────────────────────────── -->
  <header>
    <span class="title">VANTTEC USV</span>
    <div class="connect-row">
      <input bind:value={urlInput} placeholder="ws://JETSON_IP:9090" />
      <button on:click={handleConnect}>Connect</button>
      <span class="dot" class:green={$connected} class:red={!$connected}></span>
      <span class="conn-label">{$connected ? 'Connected' : 'Disconnected'}</span>
    </div>
  </header>

  <div class="grid">

    <!-- ── hardware health ───────────────────────────────────── -->
    <section>
      <h2>Hardware</h2>
      <div class="indicators">
        <Indicator label="Camera"  on={status.camera_status} />
        <Indicator label="Lidar"   on={status.lidar_status} />
        <Indicator label="CAN/STM" on={status.can_stm_status} />
      </div>
    </section>

    <!-- ── operation mode ───────────────────────────────────── -->
    <section>
      <h2>Operation Mode</h2>
      <span class="badge" class:auto={status.op_mode===0}
                          class:teleop={status.op_mode===1}
                          class:inactive={status.op_mode===2}>
        {opLabel}
      </span>
    </section>

    <!-- ── localization ──────────────────────────────────────── -->
    <section>
      <h2>Localization</h2>
      <div class="kv-grid">
        <span>EKF</span>
        <span class="badge {ekfQuality}">{ekfLabel}</span>
        <span>GPS Pos</span>
        <span class="badge {gpsQuality}">{gpsLabel}</span>
        <span>HDT Status</span>
        <span class="mono">{status.gps_hdt_status}</span>
      </div>
    </section>

    <!-- ── pose ─────────────────────────────────────────────── -->
    <section>
      <h2>Pose</h2>
      <div class="kv-grid">
        <span>X</span>    <span class="mono">{pose.x.toFixed(3)} m</span>
        <span>Y</span>    <span class="mono">{pose.y.toFixed(3)} m</span>
        <span>θ</span>    <span class="mono">{(pose.theta * 180 / Math.PI).toFixed(2)}°</span>
      </div>
    </section>

    <!-- ── mission ───────────────────────────────────────────── -->
    <section>
      <h2>Mission</h2>
      <div class="kv-grid">
        <span>ID</span>     <span class="mono">{status.mission_id}</span>
        <span>State</span>  <span class="mono">{status.mission_state}</span>
        <span>Status</span>
        <span class="badge" class:good={status.mission_status===1}
                            class:bad={status.mission_status===0}>
          {status.mission_status === 1 ? 'COMPLETE' : 'RUNNING'}
        </span>
      </div>
    </section>

    <!-- ── camera feed ──────────────────────────────────────── -->
    <CameraPanel {ros} />

    <!-- ── objects ───────────────────────────────────────────── -->
    <section class="wide">
      <h2>Detected Objects ({status.obj_list?.length ?? 0})</h2>
      {#if status.obj_list?.length > 0}
        <table>
          <thead>
            <tr><th>#</th><th>Type</th><th>Color</th><th>X (m)</th><th>Y (m)</th><th>UUID</th></tr>
          </thead>
          <tbody>
            {#each status.obj_list as obj, i}
              <tr>
                <td>{i + 1}</td>
                <td>{obj.type}</td>
                <td>
                  <span class="color-dot" style="background:{COLOR_HEX[obj.color] ?? '#888'}"></span>
                  {COLOR_NAMES[obj.color] ?? obj.color}
                </td>
                <td class="mono">{obj.x.toFixed(3)}</td>
                <td class="mono">{obj.y.toFixed(3)}</td>
                <td class="mono uuid">{obj.uuid}</td>
              </tr>
            {/each}
          </tbody>
        </table>
      {:else}
        <p class="empty">No objects detected</p>
      {/if}
    </section>

  </div>
</main>

<style>
  :global(body) {
    margin: 0;
    background: #0f1117;
    color: #e2e8f0;
    font-family: 'Inter', system-ui, sans-serif;
    font-size: 14px;
  }

  main { padding: 16px; }

  header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    background: #1e2130;
    padding: 10px 16px;
    border-radius: 8px;
    margin-bottom: 16px;
    border: 1px solid #2d3148;
  }

  .title { font-size: 18px; font-weight: 700; letter-spacing: 0.05em; color: #7dd3fc; }

  .connect-row { display: flex; align-items: center; gap: 8px; }
  .connect-row input {
    background: #0f1117; border: 1px solid #2d3148; color: #e2e8f0;
    padding: 4px 8px; border-radius: 4px; width: 220px;
  }
  .connect-row button {
    background: #3b82f6; color: white; border: none;
    padding: 5px 14px; border-radius: 4px; cursor: pointer;
  }
  .connect-row button:hover { background: #2563eb; }

  .dot {
    width: 10px; height: 10px; border-radius: 50%;
    display: inline-block;
  }
  .dot.green { background: #22c55e; box-shadow: 0 0 6px #22c55e; }
  .dot.red   { background: #ef4444; }

  .grid {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(260px, 1fr));
    gap: 12px;
  }

  section {
    background: #1e2130;
    border: 1px solid #2d3148;
    border-radius: 8px;
    padding: 14px 16px;
  }
  section.wide { grid-column: 1 / -1; }

  h2 { margin: 0 0 12px 0; font-size: 12px; text-transform: uppercase;
       letter-spacing: 0.08em; color: #7dd3fc; }

  .indicators { display: flex; gap: 16px; flex-wrap: wrap; }

  .kv-grid {
    display: grid;
    grid-template-columns: max-content 1fr;
    gap: 6px 12px;
    align-items: center;
  }

  .badge {
    display: inline-block;
    padding: 2px 10px;
    border-radius: 4px;
    font-size: 12px;
    font-weight: 600;
    background: #2d3148;
  }
  .badge.good, .badge.auto     { background: #14532d; color: #86efac; }
  .badge.warn, .badge.teleop   { background: #713f12; color: #fde68a; }
  .badge.bad,  .badge.inactive { background: #450a0a; color: #fca5a5; }

  .mono { font-family: 'JetBrains Mono', monospace; font-size: 13px; }

  .conn-label { font-size: 12px; color: #94a3b8; }

  table { width: 100%; border-collapse: collapse; font-size: 13px; }
  th { text-align: left; padding: 6px 10px; color: #94a3b8;
       border-bottom: 1px solid #2d3148; font-weight: 500; }
  td { padding: 6px 10px; border-bottom: 1px solid #1a1f2e; }
  tr:last-child td { border-bottom: none; }
  tr:hover td { background: #252840; }

  .color-dot {
    display: inline-block; width: 10px; height: 10px;
    border-radius: 50%; margin-right: 6px; vertical-align: middle;
  }

  .uuid { color: #64748b; font-size: 11px; }

  .empty { color: #475569; font-style: italic; margin: 4px 0; }
</style>
