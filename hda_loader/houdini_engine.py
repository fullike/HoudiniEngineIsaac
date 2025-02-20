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

import hapi
import sys

from enum import Enum

class SessionType(Enum):
    InProcess = 1
    NewNamedPipe = 2
    NewTCPSocket = 3
    ExistingNamedPipe = 4
    ExistingTCPSocket = 5
    ExistingSharedMemory = 6


class HoudiniEngineManager(object):
    DEFAULT_NAMED_PIPE = "hapi"
    DEFAULT_HOST_NAME = "127.0.0.1"
    DEFAULT_TCP_PORT = 9090

    def __init__(self):
        self.session = None
        self.cook_options = None
        self.session_type = SessionType.InProcess
        self.named_pipe = self.DEFAULT_NAMED_PIPE
        self.tcp_port = self.DEFAULT_TCP_PORT

    def startSession(self, session_type, named_pipe, tcp_port, shared_mem_name="", log_file="./he_log.txt"):
        ''' Creates a new session'''
        # Only start a new Session if we dont already have a valid one
        if self.session and hapi.isSessionValid(self.session):
            return True

        # Clear the connection error before starting a new session
        hapi.clearConnectionError()

        # Init the thrift server options
        server_options = hapi.ThriftServerOptions()
        server_options.autoClose = True
        server_options.timeoutMs = 3000.0

        self.session_type = session_type
        self.named_pipe = named_pipe
        self.tcp_port = tcp_port

        if session_type == SessionType.InProcess.value:
            # In-Process HAPI
            print("Creating a HAPI in-process session...")
            session_info = hapi.SessionInfo()
            self.session = hapi.createInProcessSession(session_info)
        elif session_type == SessionType.NewNamedPipe.value:
            # Start our named-pipe server
            print("Starting a named-pipe server...")
            hapi.startThriftNamedPipeServer(
                server_options, named_pipe, log_file)

            # Connect to the newly started server
            print("Connecting to the named-pipe session...")
            session_info = hapi.SessionInfo()
            self.session = hapi.createThriftNamedPipeSession(
                named_pipe, session_info)
        elif session_type == SessionType.NewTCPSocket.value:
            # Start our socket server
            print("Starting a TCP socket server...")
            hapi.startThriftSocketServer(server_options, tcp_port, log_file)

            # Connect to the newly started server
            print("Connecting to the TCP socket session...")
            session_info = hapi.SessionInfo()
            self.session = hapi.createThriftSocketSession(
                HoudiniEngineManager.DEFAULT_HOST_NAME, tcp_port, session_info)
        elif session_type == SessionType.ExistingNamedPipe.value:
            # Existing named-pipe
            print("Connecting to an existing HAPI named pipe session...")
            session_info = hapi.SessionInfo()
            self.session = hapi.createThriftNamedPipeSession(
                named_pipe, session_info)
        elif session_type == SessionType.ExistingTCPSocket.value:
            # Existing socket server
            print("Connecting to an existing HAPI TCP socket session...")
            session_info = hapi.SessionInfo()
            self.session = hapi.createThriftSocketSession(
                HoudiniEngineManager.DEFAULT_HOST_NAME, tcp_port, session_info)
        elif session_type == SessionType.ExistingSharedMemory.value:
            # Shared memory session
            print("Connecting to an existing HAPI shared memory session...")
            session_info = hapi.SessionInfo()
            self.session = hapi.CreateThriftSharedMemorySession(
                shared_mem_name, session_info)
        else:
            print("Cannot connect to unknown session type ({})".format(session_type))
            return False

        if not self.session:
            connectionError = self.getConnectionError()
            if connectionError:
                print(
                    "Houdini Engine Session failed to connect - {}".format(connectionError))
            return False

        return True

    def restartSession(self, session_type, use_cooking_thread):
        '''Stop the existing session if valid, and creates a new session'''

        print("Restarting the Houdini Engine session...\n")

        # Make sure we stop the current session if it is still valid
        self.stopSession()

        success = False
        if not self.startSession(session_type, self.named_pipe, self.tcp_port):
            print(
                "Failed to restart the Houdini Engine session - Failed to start the new Session")
        else:
            # Now initialize HAPI with this session
            if not self.initializeHAPI(use_cooking_thread):
                print(
                    "Failed to restart the Houdini Engine session - Failed to initialize HAPI")
            else:
                success = True

        return success

    def stopSession(self):
        '''Cleanup and shutdown an existing session'''
        if self.session and hapi.isSessionValid(self.session):
            # SessionPtr is valid, clean up and close the session
            print("\nCleaning up and closing session...")

            hapi.cleanup(self.session)

            # When using an in-process session, this method must be called
            # in order for the host process to shutdown cleanly.
            if self.session_type == SessionType.InProcess:
                hapi.shutdown(self.session)

            hapi.closeSession(self.session)

        return True

    def initializeHAPI(self, use_cooking_thread):
        '''Initializes the HAPI session, should be called after successfully creating a session'''
        # We need a valid Session
        if self.session is None or not hapi.isSessionValid(self.session):
            print("Failed to initialize HAPI: The session is invalid.")
            return False

        # TODO: Currently throwing a hapi.NotInitializedError
        # if hapi.isInitialized(self.session) == hapi.result.NotInitialized:

        # Initialize HAPI
        self.cook_options = hapi.CookOptions()

        self.cook_options.curveRefineLOD = 8.0
        self.cook_options.clearErrorsAndWarnings = False
        self.cook_options.maxVerticesPerPrimitive = 3
        self.cook_options.splitGeosByGroup = False
        self.cook_options.refineCurveToLinear = True
        self.cook_options.handleBoxPartTypes = False
        self.cook_options.handleSpherePartTypes = False
        self.cook_options.splitPointsByVertexAttributes = False
        self.cook_options.packedPrimInstancingMode = hapi.packedPrimInstancingMode.Flat

        success = hapi.initialize(
            self.session,
            self.cook_options,
            use_cooking_thread,
            -1,                     # cooking_thread_stack_size
            "",                     # houdini_environment_files
            None,                   # otl_search_path
            None,                   # dso_search_path
            None,                   # image_dso_search_path
            None                    # audio_dso_search_path
        )

        if not success:
            print("Houdini Engine API initialization failed")
            return False

        print("Successfully initialized Houdini Engine.")
        return True

    def getSession(self):
        '''Get the HAPI session'''
        return self.session

    def getCookOptions(self):
        '''Get the cook options used to initialize the HAPI session'''
        return self.cook_options

    def loadAsset(self, otl_path):
        '''Load a new HDA asset'''

        if self.getSession() is None:
            return False

        # Load the library from file
        print("Loading asset...")
        asset_library_id = hapi.loadAssetLibraryFromFile(
            self.session, otl_path, False)

        asset_count = hapi.getAvailableAssetCount(
            self.session, asset_library_id)

        if asset_count > 1:
            print("Should only be loading 1 asset here")
            return

        asset_names_array = hapi.getAvailableAssets(
            self.session, asset_library_id, asset_count)
        asset_name = self.getString(asset_names_array[0])

        print("  Loaded: {}".format(asset_name))
        node_id = hapi.createNode(self.session, -1, asset_name, "hei_label", False)
        return node_id

    def unloadAsset(self, node_id):
        hapi.deleteNode(self.session, node_id)
        hapi.cleanup()

    def _waitForCook(self):
        if self.session is None:
            return False

        status = status = hapi.getStatus(self.session, hapi.statusType.CookState)
        while (status in [hapi.state.StartingCook, hapi.state.Cooking]):
            status = hapi.getStatus(self.session, hapi.statusType.CookState)

        if status != hapi.state.Ready:
            print("Cook failure: {}".format(self.getLastCookError()))
            return False

        return True

    def cookNode(self, node_id):
        '''cook node_id directly can only cook the first output even set cook_options.preferOutputNodes to True'''
        #hapi.cookNode(self.session, node_id, self.cook_options)
        #self._waitForCook()
        node_info = hapi.getNodeInfo(self.session, node_id)
        outputs = []
        for i in range(node_info.outputCount):
            output = hapi.getOutputNodeId(self.session, node_id, i)
            outputs +=[output]
            hapi.cookNode(self.session, output, self.cook_options)
        return outputs

    def getParameters(self, node_id):
        node_info = hapi.getNodeInfo(self.session, node_id)
        parm_infos = hapi.getParameters(self.session, node_id, 0, node_info.parmCount)
        parms = dict()
        
        def get_parent(info):
            path = []
            while info.parentId != -1:
                info = parm_infos[info.parentId]
                path = [self.getString(info.nameSH)] + path
            parent = parms
            for name in path:
                parent = parent[name]
            return parent
        
        for i in range(node_info.parmCount):
            info = parm_infos[i]
            parent = get_parent(info)
            name = self.getString(info.nameSH)
            if info.type == hapi.parmType.Folder:
                parent[name] = dict()
            elif info.type == hapi.parmType.Int:
                parent[name] = hapi.getParmIntValues(self.session, node_id, info.intValuesIndex, info.size) if info.size > 1 else hapi.getParmIntValue(self.session, node_id, name, 0)
            elif info.type == hapi.parmType.Toggle:
                parent[name] = bool(hapi.getParmIntValues(self.session, node_id, info.intValuesIndex, info.size) if info.size > 1 else hapi.getParmIntValue(self.session, node_id, name, 0)) 
            elif parm_infos[i].type == hapi.parmType.Float:
                parent[name] = hapi.getParmFloatValues(self.session, node_id, info.floatValuesIndex, info.size) if info.size > 1 else hapi.getParmFloatValue(self.session, node_id, name, 0)
            elif parm_infos[i].type == hapi.parmType.String:
                if info.size > 1:
                    handles = hapi.getParmStringValues(self.session, node_id, True, info.stringValuesIndex, info.size) 
                    values = []
                    for v in range(info.size):
                        values += self.getString(handles[v])
                    parent[name] = values
                else:
                    parent[name] = self.getString(hapi.getParmStringValue(self.session, node_id, name, 0, True))
        return parms

    def setParameters(self, node_id, parms):
        for name, value in parms.items():
            parm_id = hapi.getParmIdFromName(self.session, node_id, name)
            if isinstance(value, int):
                hapi.setParmIntValue(self.session, node_id, name, 0, value)
            elif isinstance(value, bool):
                hapi.setParmIntValue(self.session, node_id, name, 0, int(value))
            elif isinstance(value, float):
                hapi.setParmFloatValue(self.session, node_id, name, 0, value)
            elif isinstance(value, str):
                hapi.setParmStringValue(self.session, node_id, value, parm_id, 0)

    def getAttribute(self, owner, node_id, part_id, attr_name):
        attr_info = hapi.getAttributeInfo(self.session, node_id, part_id, attr_name, owner,)
        values = []
        if attr_info.storage == hapi.storageType.Int:
            for v in range(attr_info.count):
                values +=[hapi.getAttributeIntData(self.session, node_id, part_id, attr_name, attr_info, -1, v, 1)]
        elif attr_info.storage == hapi.storageType.IntArray:
            data, counts = hapi.getAttributeIntArrayData(self.session, node_id, part_id, attr_name, attr_info, attr_info.totalArrayElements, 0, attr_info.count)
            start = 0
            for v in range(attr_info.count):
                values +=[data[start:start+counts[v]]]
                start +=counts[v]           
        elif attr_info.storage == hapi.storageType.Float:
            for v in range(attr_info.count):
                values +=[hapi.getAttributeFloatData(self.session, node_id, part_id, attr_name, attr_info, -1, v, 1)]
        elif attr_info.storage == hapi.storageType.String:
            handles = hapi.getAttributeStringData(self.session, node_id, part_id, attr_name, attr_info, 0, attr_info.count) 
            for v in range(attr_info.count):
                values += [self.getString(handles[v])]
        return values

    def getAttributes(self, owner, node_id, part_id):
        part_info = hapi.getPartInfo(self.session, node_id, part_id)
        point_attr_count = part_info.attributeCounts[owner]
        point_attr_nameSH = hapi.getAttributeNames(self.session, node_id, part_id, owner, point_attr_count)
        attribs = dict()
        for i in range(point_attr_count):
            attr_name = self.getString(point_attr_nameSH[i])
            attribs[attr_name] = self.getAttribute(owner, node_id, part_id, attr_name)
        return attribs

    def readPoints(self, node_id):
        part_id = 0
        part_info = hapi.getPartInfo(self.session, node_id, part_id)
        point_attr_count = part_info.attributeCounts[hapi.attributeOwner.Point]
        point_attr_nameSH = hapi.getAttributeNames(self.session, node_id, part_id, hapi.attributeOwner.Point, point_attr_count)
        attribs = dict()
        for i in range(point_attr_count):
            attr_name = self.getString(point_attr_nameSH[i])
            attr_info = hapi.getAttributeInfo(self.session, node_id, part_id, attr_name, hapi.attributeOwner.Point,)
            if attr_info.storage == hapi.storageType.Int:
                values = []
                for v in range(attr_info.count):
                    values +=[hapi.getAttributeIntData(self.session, node_id, part_id, attr_name, attr_info, -1, v, 1)]
                attribs[attr_name] = values
            elif attr_info.storage == hapi.storageType.IntArray:
                data, counts = hapi.getAttributeIntArrayData(self.session, node_id, part_id, attr_name, attr_info, attr_info.totalArrayElements, 0, attr_info.count)
                values = []
                start = 0
                for v in range(attr_info.count):
                    values +=[data[start:start+counts[v]]]
                    start +=counts[v]
                attribs[attr_name] = values                
            elif attr_info.storage == hapi.storageType.Float:
                values = []
                for v in range(attr_info.count):
                    values +=[hapi.getAttributeFloatData(self.session, node_id, part_id, attr_name, attr_info, -1, v, 1)]
                attribs[attr_name] = values
            elif attr_info.storage == hapi.storageType.String:
                handles = hapi.getAttributeStringData(self.session, node_id, part_id, attr_name, attr_info, 0, attr_info.count) 
                values = []
                for v in range(attr_info.count):
                    values += [self.getString(handles[v])]
                attribs[attr_name] = values
        return attribs

    def readGeometry(self, node_id):
        '''Read mesh data from Houdini for processing'''
        # Get mesh geo info.
        meshes = []
        geo_info = hapi.getGeoInfo(self.session, node_id)
        for i in range(geo_info.partCount):
            part_info = hapi.getPartInfo(self.session, geo_info.nodeId, i)
            if part_info.isInstanced:
                mesh = dict()
                mesh['pos'] = part_attrs['P']
                mesh['indices'] = hapi.getVertexList(self.session, geo_info.nodeId, part_info.id, 0, part_info.vertexCount)
                mesh['P'] = self.getAttribute(hapi.attributeOwner.Point, node_id, part_info.id, "P")
                mesh['name'] = self.getAttribute(hapi.attributeOwner.Prim, node_id, part_info.id, "name")
                mesh['material'] = self.getAttribute(hapi.attributeOwner.Prim, node_id, part_info.id, "material")
                meshes.append(mesh)
            else:
                part_attrs = self.getAttributes(hapi.attributeOwner.Prim, node_id, part_info.id)
        return meshes

    def getLastError(self):
        '''Helper method to retrieve the last error message'''
        buffer_length = hapi.getStatusStringBufLength(
            self.session,
            hapi.statusType.CallResult,
            hapi.statusVerbosity.Errors)

        if buffer_length <= 1:
            return ""

        string_val = hapi.getStatusString(
            self.session, hapi.statusType.CallResult, buffer_length)

        return string_val

    def getLastCookError(self):
        '''Helper method to retrieve the last cook error message'''
        buffer_length = 0
        buffer_length = hapi.getStatusStringBufLength(
            self.session,
            hapi.statusType.CookResult,
            hapi.statusVerbosity.Errors)

        if buffer_length <= 1:
            return ""

        string_val = hapi.getStatusString(
            self.session, hapi.statusType.CookResult, buffer_length)

        return string_val

    def getConnectionError(self):
        '''Helper method to retrieve the last connection error message'''
        buffer_length = hapi.getConnectionErrorLength()

        if buffer_length <= 1:
            return ""

        string_val = hapi.getConnectionError(buffer_length, True)
        return string_val

    def getString(self, string_handle):
        '''Helper method to retrieve a string from a HAPI_StringHandle'''
        buffer_length = hapi.getStringBufLength(self.session, string_handle)

        string_val = hapi.getString(self.session, string_handle, buffer_length)
        return string_val

    def saveToHip(self, filename):
        '''Save the session to a .hip file in the application directory'''
        success = hapi.saveHIPFile(self.session, filename, False)
        return success