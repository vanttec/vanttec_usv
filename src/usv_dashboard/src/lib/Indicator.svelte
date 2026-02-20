<script>
  export let label = ''
  export let on = false
</script>

<div class="indicator">
  <span class="led" class:on class:off={!on}></span>
  <span class="label">{label}</span>
</div>

<style>
  .indicator { display: flex; align-items: center; gap: 8px; }

  .led {
    width: 12px; height: 12px; border-radius: 50%;
    flex-shrink: 0;
  }
  .led.on  { background: #22c55e; box-shadow: 0 0 8px #22c55e; }
  .led.off { background: #374151; }

  .label { font-size: 13px; }
</style>
