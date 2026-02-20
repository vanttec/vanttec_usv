<script>
  import { onDestroy } from 'svelte'
  import ROSLIB from 'roslib'

  export let ros = null   // ROSLIB.Ros instance, passed from parent

  let enabled = false
  let topic = null
  let fps = 0
  let frameCount = 0
  let fpsInterval
  let canvas
  let status = 'idle'   // idle | waiting | streaming | error

  // Raw sensor_msgs/Image → canvas via base64 decode + BGR→RGB swap
  function renderRawImage(msg) {
    if (!canvas) return

    const { width, height, encoding, data: b64 } = msg

    // Decode base64 to byte array
    const binary = atob(b64)
    const bytes = new Uint8Array(binary.length)
    for (let i = 0; i < binary.length; i++) bytes[i] = binary.charCodeAt(i)

    const ctx = canvas.getContext('2d')
    canvas.width  = width
    canvas.height = height

    const imgData = ctx.createImageData(width, height)
    const out = imgData.data  // RGBA

    // Handle common ZED encodings: bgr8, rgb8, bgra8, rgba8
    const isBGR  = encoding === 'bgr8'  || encoding === 'bgra8'
    const hasAlpha = encoding === 'rgba8' || encoding === 'bgra8'
    const srcChannels = hasAlpha ? 4 : 3

    for (let i = 0; i < width * height; i++) {
      const s = i * srcChannels
      const d = i * 4
      out[d + 0] = isBGR ? bytes[s + 2] : bytes[s + 0]  // R
      out[d + 1] = bytes[s + 1]                          // G
      out[d + 2] = isBGR ? bytes[s + 0] : bytes[s + 2]  // B
      out[d + 3] = hasAlpha ? bytes[s + 3] : 255         // A
    }

    ctx.putImageData(imgData, 0, 0)
    frameCount++
    status = 'streaming'
  }

  function onToggle() {
    if (enabled) {
      startStream()
    } else {
      stopStream()
    }
  }

  function startStream() {
    if (!ros) {
      status = 'error'
      return
    }

    status = 'waiting'

    // Subscribe to compressed first — needs `image_transport republish` running on Jetson
    // Falls back gracefully: if no messages arrive, the waiting state stays visible
    topic = new ROSLIB.Topic({
      ros,
      name: '/bebblebrox/video/compressed',
      messageType: 'sensor_msgs/msg/CompressedImage',
      queue_length: 1,
    })

    topic.subscribe((msg) => {
      if (!canvas) return
      // CompressedImage: data is base64 JPEG/PNG
      const imgEl = new Image()
      imgEl.onload = () => {
        canvas.width  = imgEl.width
        canvas.height = imgEl.height
        canvas.getContext('2d').drawImage(imgEl, 0, 0)
        frameCount++
        status = 'streaming'
      }
      imgEl.src = `data:image/${msg.format.split(';')[0]};base64,${msg.data}`
    })

    fpsInterval = setInterval(() => {
      fps = frameCount
      frameCount = 0
      // If no frames received after connect, show hint
      if (status === 'waiting') status = 'waiting'
    }, 1000)
  }

  function stopStream() {
    if (topic) {
      topic.unsubscribe()
      topic = null
    }
    clearInterval(fpsInterval)
    fps = 0
    frameCount = 0
    status = 'idle'
    if (canvas) {
      canvas.getContext('2d').clearRect(0, 0, canvas.width, canvas.height)
    }
  }

  onDestroy(stopStream)
</script>

<section class="wide">
  <h2>
    Camera Feed
    <label class="toggle">
      <input type="checkbox" bind:checked={enabled} on:change={onToggle} />
      <span>Enable</span>
    </label>
    {#if enabled && status === 'streaming'}
      <span class="fps">{fps} fps</span>
    {/if}
    {#if enabled && status === 'waiting'}
      <span class="hint">
        No frames — run on Jetson:
        <code>ros2 run image_transport republish raw compressed --ros-args -r in:=/bebblebrox/video/image -r out/compressed:=/bebblebrox/video/compressed</code>
      </span>
    {/if}
  </h2>

  {#if enabled}
    <div class="frame-wrap">
      <canvas bind:this={canvas}></canvas>
    </div>
    {#if status === 'idle' || status === 'waiting'}
      <p class="empty">
        {status === 'idle' ? 'Waiting for connection…' : 'Waiting for frames on /bebblebrox/video/compressed…'}
      </p>
    {/if}
  {:else}
    <p class="empty">Enable to start receiving camera frames</p>
  {/if}
</section>

<style>
  section {
    background: #1e2130;
    border: 1px solid #2d3148;
    border-radius: 8px;
    padding: 14px 16px;
  }
  section.wide { grid-column: 1 / -1; }

  h2 {
    margin: 0 0 12px 0;
    font-size: 12px;
    text-transform: uppercase;
    letter-spacing: 0.08em;
    color: #7dd3fc;
    display: flex;
    align-items: center;
    gap: 14px;
    flex-wrap: wrap;
  }

  .toggle {
    display: flex;
    align-items: center;
    gap: 6px;
    cursor: pointer;
    font-size: 12px;
    text-transform: none;
    letter-spacing: 0;
    color: #94a3b8;
  }
  .toggle input { cursor: pointer; accent-color: #3b82f6; }

  .fps {
    margin-left: auto;
    font-size: 11px;
    color: #64748b;
    font-family: monospace;
    text-transform: none;
    letter-spacing: 0;
  }

  .hint {
    font-size: 11px;
    color: #f59e0b;
    text-transform: none;
    letter-spacing: 0;
    font-weight: normal;
  }
  .hint code {
    display: block;
    margin-top: 4px;
    background: #0f1117;
    padding: 4px 8px;
    border-radius: 4px;
    font-size: 10px;
    color: #e2e8f0;
    white-space: pre-wrap;
    word-break: break-all;
  }

  .frame-wrap {
    width: 100%;
    display: flex;
    justify-content: center;
    background: #0f1117;
    border-radius: 4px;
    overflow: hidden;
    min-height: 40px;
  }
  canvas {
    max-width: 100%;
    max-height: 480px;
    object-fit: contain;
    display: block;
  }

  .empty { color: #475569; font-style: italic; margin: 4px 0; font-size: 13px; }
</style>
