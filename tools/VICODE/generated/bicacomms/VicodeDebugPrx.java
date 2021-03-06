// **********************************************************************
//
// Copyright (c) 2003-2013 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************
//
// Ice version 3.5.1
//
// <auto-generated>
//
// Generated from file `BicaIceComms.ice'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

package bicacomms;

public interface VicodeDebugPrx extends Ice.ObjectPrx
{
    public int getState(String id);

    public int getState(String id, java.util.Map<String, String> __ctx);

    public Ice.AsyncResult begin_getState(String id);

    public Ice.AsyncResult begin_getState(String id, java.util.Map<String, String> __ctx);

    public Ice.AsyncResult begin_getState(String id, Ice.Callback __cb);

    public Ice.AsyncResult begin_getState(String id, java.util.Map<String, String> __ctx, Ice.Callback __cb);

    public Ice.AsyncResult begin_getState(String id, Callback_VicodeDebug_getState __cb);

    public Ice.AsyncResult begin_getState(String id, java.util.Map<String, String> __ctx, Callback_VicodeDebug_getState __cb);

    public int end_getState(Ice.AsyncResult __result);

    public ComponentsList getListComponents();

    public ComponentsList getListComponents(java.util.Map<String, String> __ctx);

    public Ice.AsyncResult begin_getListComponents();

    public Ice.AsyncResult begin_getListComponents(java.util.Map<String, String> __ctx);

    public Ice.AsyncResult begin_getListComponents(Ice.Callback __cb);

    public Ice.AsyncResult begin_getListComponents(java.util.Map<String, String> __ctx, Ice.Callback __cb);

    public Ice.AsyncResult begin_getListComponents(Callback_VicodeDebug_getListComponents __cb);

    public Ice.AsyncResult begin_getListComponents(java.util.Map<String, String> __ctx, Callback_VicodeDebug_getListComponents __cb);

    public ComponentsList end_getListComponents(Ice.AsyncResult __result);
}
