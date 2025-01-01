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


def getLastError(session):
    '''Helper method to retrieve the last error message'''
    buffer_length = hapi.getStatusStringBufLength(
        session,
        hapi.statusType.CallResult,
        hapi.statusVerbosity.Errors)

    if buffer_length <= 1:
        return ""

    string_val = hapi.getStatusString(
        session, hapi.statusType.CallResult, buffer_length)

    return string_val

def getLastCookError(session):
    '''Helper method to retrieve the last cook error message'''
    buffer_length = 0
    buffer_length = hapi.getStatusStringBufLength(
        session,
        hapi.statusType.CookResult,
        hapi.statusVerbosity.Errors)

    if buffer_length <= 1:
        return ""

    string_val = hapi.getStatusString(
        session, hapi.statusType.CookResult, buffer_length)

    return string_val

def getConnectionError():
    '''Helper method to retrieve the last connection error message'''
    buffer_length = hapi.getConnectionErrorLength()

    if buffer_length <= 1:
        return ""

    string_val = hapi.getConnectionError(buffer_length, True)
    return string_val

def getString(session, string_handle):
    '''Helper method to retrieve a string from a HAPI_StringHandle'''
    buffer_length = hapi.getStringBufLength(session, string_handle)

    string_val = hapi.getString(session, string_handle, buffer_length)
    return string_val

def saveToHip(session, filename):
    '''Save the session to a .hip file in the application directory'''
    success = hapi.saveHIPFile(session, filename, False)
    return success
