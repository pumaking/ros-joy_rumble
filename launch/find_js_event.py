#!/usr/bin/env python

import os

input_dir = "/dev/input"
sys_input_dir = "/sys/class/input"
dev_type = "js"

def get_all_dev():
  return list(filter(lambda s: s.startswith(dev_type), os.listdir(sys_input_dir)))

def get_dev_dir(dev_name):
  dev_dir = os.path.join(sys_input_dir, dev_name)
  if not os.path.isdir(dev_dir):
    raise Exception("Could not find %s; %s is not a directory." % (dev_name, dev_dir))

  device_dir = os.path.join(dev_dir, "device")

  if not os.path.isdir(device_dir):
    raise Exception("ERROR: Expected %s to contain directory device." % dev_dir)

  return device_dir

def get_possible_joys(devs):
  poss = []
  for dev in devs:
    name_file = os.path.join(get_dev_dir(dev), "name")
    if not os.path.isfile(name_file):
      raise Exception("ERROR: Could not name of %s" % dev)

    name = open(name_file).read()
    if "mouse" not in name and "Mouse" not in name:
      poss.append(dev)

  return poss

def get_dev_event(dev_name):
  device_dir = get_dev_dir(dev_name)

  for name in os.listdir(device_dir):
    if name.startswith("event"):
      event_dir = os.path.join(input_dir, name)
      if not os.path.exists(input_dir):
        raise Exception("ERROR: Found %s for %s but %s does not exist." % (name, dev_name, event_dir))

      return event_dir

  raise Exception("ERROR: Found %s device dir, but it had no eventXX." % (dev_name))

if __name__ == "__main__":
  import sys

  if len(sys.argv) <= 1:
    poss = get_possible_joys(get_all_dev())
    if not poss:
      raise Exception("ERROR: No devices found.")

    print(get_dev_event(poss[0]))

  else:
    print(get_dev_event(sys.argv[1]))
