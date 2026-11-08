import navigation.waypoints as wp

import shlex
import sys

import navigation.pathfinder


class WaypointController:
    def __init__(self):
        self.running = True

    def run(self):
        print("RIVAL final project Semi Autonomous Controller")
        print("Type 'help' for commands")

        while self.running:
            try: 
                command = input("\n>").strip()
                if not command:
                    print("Invalid command")
                    continue
                self.execute(command)
            except KeyboardInterrupt:
                print("\nStopping Controller")
                self.stop()

    def execute(self, command:str):
        args = shlex.split(command)
        if not args:
            return
        cmd = args[0].lower()
        if cmd == "moveto":
            self.handleMoveTo(args[1:])
        elif cmd == "center":
            self.handleCenter()
        elif cmd == "scan":
            self.handleScan()
        elif cmd == "list":
            self.handleList()
        elif cmd == "graph":
            self.handleGraph()
        elif cmd == "position":
            self.handlePosition()
        elif cmd == "help":
            self.handleHelp()

    def handleMoveTo(self, args: list[str]):
        return
    def handleScan():
        return
    def handleList():
        return  
    def handleGraph():
        print("Waypoint ZERO: wp[16] = [0, 0]")
        print()

        print("Regular Waypoints:")
        for i in range(12):
            print(f"    wp[{i}] \t:[{wp.waypoints[i][0]}][{wp.waypoints[i][1]}]")
        print("Aruco Waypoints:")
        for i in range(12, 16):
            print(f"    wp[{i}] \t:[{wp.waypoints[i][0]}][{wp.waypoints[i][1]}]")

    def handlePos():
        return
    def handleHelp():
        print("[1] move into waypoints (autonomous):") 
        print("[2] move into center (autonomous)")
        print("[3] visit and scans all aruco (autonomous)")
        print("miscellaneous:")
        print("[l] waypoint lists")

def main():

