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

public abstract class Callback_VicodeDebug_getState extends Ice.TwowayCallback
{
    public abstract void response(int __ret);

    public final void __completed(Ice.AsyncResult __result)
    {
        VicodeDebugPrx __proxy = (VicodeDebugPrx)__result.getProxy();
        int __ret = 0;
        try
        {
            __ret = __proxy.end_getState(__result);
        }
        catch(Ice.LocalException __ex)
        {
            exception(__ex);
            return;
        }
        response(__ret);
    }
}
