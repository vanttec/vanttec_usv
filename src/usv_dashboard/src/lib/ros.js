import ROSLIB from 'roslib'
import { writable } from 'svelte/store'

export const connected = writable(false)
export const rosUrl = writable('ws://localhost:9090')

let ros = null

export function connect(url) {
  if (ros) {
    ros.close()
  }

  ros = new ROSLIB.Ros({ url })

  ros.on('connection', () => connected.set(true))
  ros.on('error',      () => connected.set(false))
  ros.on('close',      () => connected.set(false))

  return ros
}

export function subscribe(ros, topic, type, callback) {
  const t = new ROSLIB.Topic({ ros, name: topic, messageType: type })
  t.subscribe(callback)
  return t
}
