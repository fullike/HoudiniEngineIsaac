# Copyright (c) <2023> Side Effects Software Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. The name of Side Effects Software may not be used to endorse or
#    promote products derived from this software without specific prior
#    written permission.
#
# THIS SOFTWARE IS PROVIDED BY SIDE EFFECTS SOFTWARE "AS IS" AND ANY EXPRESS
# OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
# OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
# NO EVENT SHALL SIDE EFFECTS SOFTWARE BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import os, sys

# import Python Libs and add Path for the Dll Files 
sys.path.append("/opt/hfs20.5.445/houdini/python3.10libs")

import hou
import hapi
from he_geometry import HoudiniEngineGeometry
from he_manager import HoudiniEngineManager, SessionType
import he_utility


def printCommandMenu():
    command_menu = """
Houdini Engine Sample Commands
------------------------------
Working with HDAs
  - cook: Create & cook the hexagona sample HDA
  - parms: Fetch and print node parameters
  - attribs: Fetch and print node attributes
Working with Geometry
  - setgeo: Marshal mesh data to Houdini
  - getgeo: Read mesh data from Houdini
Working with Sessions
  - checkvalid: Check if the session is valid
General Commands
  - help: Print menu of commands
  - save: Save the Houdini session to a hip file
  - quit: Cleanup and shutdown the Houdini session"""
    print(command_menu)


def main():
    init_session_menu = """
===================================
 Houdini Engine Sample Application
===================================
Start a new Houdini Engine Session via HARS:
    1: In-Process Session
    2: Named-Pipe Session
    3: TCP Socket Session\n
Connect to an existing Houdini Engine Session via SessionSync:
    4: Existing Named-Pipe Session
    5: Existing TCP Socket Session
    6: Existing Shared Memory Session\n"""
    print(init_session_menu)
    session_type = int(input(">> "))

    he_manager = HoudiniEngineManager()
    if he_manager is None:
        print("Failed to create the Houdini Engine Manager.")
        return

    use_cooking_thread = True  # Enables asynchronous cooking of nodes.
    named_pipe = he_manager.DEFAULT_NAMED_PIPE
    tcp_port = he_manager.DEFAULT_TCP_PORT

    if session_type == SessionType.ExistingNamedPipe.value:
        print("Please specify the pipe name:")
        named_pipe = input(">> ")
    elif session_type == SessionType.ExistingTCPSocket.value:
        print("Please specify the TCP port:")
        tcp_port = int(input(">> "))
    elif session_type == SessionType.ExistingSharedMemory.value:
        print("Please specify the shared memory name:")
        shared_mem_name = input(">> ")

    if not he_manager.startSession(session_type, named_pipe, tcp_port):
        print("ERROR: Failed to create a Houdini Engine session.")
        return

    if not he_manager.initializeHAPI(use_cooking_thread):
        print("ERROR: Failed to initialize HAPI.")
        return

    print("Loading the hexagona sample HDA: ")

    otl_path = "{}/../HDA/hexagona_lite.hda".format(os.getcwd())
    asset_name = he_manager.loadAsset(otl_path)
    if asset_name is None:
        print("Failed to load the default HDA.")
        return

    printCommandMenu()
    hexagona_cook = False
    mesh_data_generated = None
    hexagona_node_id = 0
    hexagona_part_id = 0
    while True:
        user_cmd = input(">> ")
        if user_cmd == "quit":
            break

        if user_cmd == "cook":
            hexagona_cook = he_manager.createAndCookNode(
                asset_name, hexagona_node_id)
        elif user_cmd == "parms":
            if hexagona_cook:
                he_manager.getParameters(hexagona_node_id)
            else:
                print(
                    "Error: The hexagona sample HDA must be cooked before you can query its parameters (cmd cook).")
        elif user_cmd == "attribs":
            if hexagona_cook:
                he_manager.getAttributes(hexagona_node_id, hexagona_part_id)
            else:
                print(
                    "Error: The hexagona sample HDA must be cooked before you can query its attributes (cmd cook).")
        elif user_cmd == "setgeo":
            mesh_data_generated = HoudiniEngineGeometry.sendGeometryToHoudini(
                he_manager.session, he_manager.cook_options)
        elif user_cmd == "getgeo":
            if mesh_data_generated is not None:
                HoudiniEngineGeometry.readGeometryFromHoudini(
                    he_manager.session, mesh_data_generated, he_manager.cook_options)
            else:
                print(
                    "Mesh data must be set and sent to Houdini to cook before it can be queried (cmd setgeo).")
        elif user_cmd == "checkvalid":
            if he_manager.getSession() is None:
                print("No session exists.")

            if hapi.isSessionValid(he_manager.session):
                print("The session is VALID.")
            else:
                print("The session is INVALID.")
        elif user_cmd == "help":
            printCommandMenu()
        elif user_cmd == "save":
            print("\nFilename (.hip) to save the session to: ", end='')
            filename = input()

            success = he_utility.saveToHip(
                he_manager.session, filename)
            if success:
                print("Hip file saved successfully.")
            else:
                print(he_utility.getLastError(he_manager.session))

    he_manager.stopSession()


if __name__ == "__main__":
    main()
