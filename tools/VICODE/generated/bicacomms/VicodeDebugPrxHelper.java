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

public final class VicodeDebugPrxHelper extends Ice.ObjectPrxHelperBase implements VicodeDebugPrx
{
    private static final String __getListComponents_name = "getListComponents";

    public ComponentsList getListComponents()
    {
        return getListComponents(null, false);
    }

    public ComponentsList getListComponents(java.util.Map<String, String> __ctx)
    {
        return getListComponents(__ctx, true);
    }

    private ComponentsList getListComponents(java.util.Map<String, String> __ctx, boolean __explicitCtx)
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        final Ice.Instrumentation.InvocationObserver __observer = IceInternal.ObserverHelper.get(this, "getListComponents", __ctx);
        int __cnt = 0;
        try
        {
            while(true)
            {
                Ice._ObjectDel __delBase = null;
                try
                {
                    __checkTwowayOnly("getListComponents");
                    __delBase = __getDelegate(false);
                    _VicodeDebugDel __del = (_VicodeDebugDel)__delBase;
                    return __del.getListComponents(__ctx, __observer);
                }
                catch(IceInternal.LocalExceptionWrapper __ex)
                {
                    __cnt = __handleExceptionWrapperRelaxed(__delBase, __ex, null, __cnt, __observer);
                }
                catch(Ice.LocalException __ex)
                {
                    __cnt = __handleException(__delBase, __ex, null, __cnt, __observer);
                }
            }
        }
        finally
        {
            if(__observer != null)
            {
                __observer.detach();
            }
        }
    }

    public Ice.AsyncResult begin_getListComponents()
    {
        return begin_getListComponents(null, false, null);
    }

    public Ice.AsyncResult begin_getListComponents(java.util.Map<String, String> __ctx)
    {
        return begin_getListComponents(__ctx, true, null);
    }

    public Ice.AsyncResult begin_getListComponents(Ice.Callback __cb)
    {
        return begin_getListComponents(null, false, __cb);
    }

    public Ice.AsyncResult begin_getListComponents(java.util.Map<String, String> __ctx, Ice.Callback __cb)
    {
        return begin_getListComponents(__ctx, true, __cb);
    }

    public Ice.AsyncResult begin_getListComponents(Callback_VicodeDebug_getListComponents __cb)
    {
        return begin_getListComponents(null, false, __cb);
    }

    public Ice.AsyncResult begin_getListComponents(java.util.Map<String, String> __ctx, Callback_VicodeDebug_getListComponents __cb)
    {
        return begin_getListComponents(__ctx, true, __cb);
    }

    private Ice.AsyncResult begin_getListComponents(java.util.Map<String, String> __ctx, boolean __explicitCtx, IceInternal.CallbackBase __cb)
    {
        __checkAsyncTwowayOnly(__getListComponents_name);
        IceInternal.OutgoingAsync __result = new IceInternal.OutgoingAsync(this, __getListComponents_name, __cb);
        try
        {
            __result.__prepare(__getListComponents_name, Ice.OperationMode.Idempotent, __ctx, __explicitCtx);
            __result.__writeEmptyParams();
            __result.__send(true);
        }
        catch(Ice.LocalException __ex)
        {
            __result.__exceptionAsync(__ex);
        }
        return __result;
    }

    public ComponentsList end_getListComponents(Ice.AsyncResult __result)
    {
        Ice.AsyncResult.__check(__result, this, __getListComponents_name);
        boolean __ok = __result.__wait();
        try
        {
            if(!__ok)
            {
                try
                {
                    __result.__throwUserException();
                }
                catch(Ice.UserException __ex)
                {
                    throw new Ice.UnknownUserException(__ex.ice_name(), __ex);
                }
            }
            IceInternal.BasicStream __is = __result.__startReadParams();
            ComponentsList __ret;
            __ret = new ComponentsList();
            __ret.__read(__is);
            __result.__endReadParams();
            return __ret;
        }
        catch(Ice.LocalException ex)
        {
            Ice.Instrumentation.InvocationObserver __obsv = __result.__getObserver();
            if(__obsv != null)
            {
                __obsv.failed(ex.ice_name());
            }
            throw ex;
        }
    }

    private static final String __getState_name = "getState";

    public int getState(String id)
    {
        return getState(id, null, false);
    }

    public int getState(String id, java.util.Map<String, String> __ctx)
    {
        return getState(id, __ctx, true);
    }

    private int getState(String id, java.util.Map<String, String> __ctx, boolean __explicitCtx)
    {
        if(__explicitCtx && __ctx == null)
        {
            __ctx = _emptyContext;
        }
        final Ice.Instrumentation.InvocationObserver __observer = IceInternal.ObserverHelper.get(this, "getState", __ctx);
        int __cnt = 0;
        try
        {
            while(true)
            {
                Ice._ObjectDel __delBase = null;
                try
                {
                    __checkTwowayOnly("getState");
                    __delBase = __getDelegate(false);
                    _VicodeDebugDel __del = (_VicodeDebugDel)__delBase;
                    return __del.getState(id, __ctx, __observer);
                }
                catch(IceInternal.LocalExceptionWrapper __ex)
                {
                    __cnt = __handleExceptionWrapperRelaxed(__delBase, __ex, null, __cnt, __observer);
                }
                catch(Ice.LocalException __ex)
                {
                    __cnt = __handleException(__delBase, __ex, null, __cnt, __observer);
                }
            }
        }
        finally
        {
            if(__observer != null)
            {
                __observer.detach();
            }
        }
    }

    public Ice.AsyncResult begin_getState(String id)
    {
        return begin_getState(id, null, false, null);
    }

    public Ice.AsyncResult begin_getState(String id, java.util.Map<String, String> __ctx)
    {
        return begin_getState(id, __ctx, true, null);
    }

    public Ice.AsyncResult begin_getState(String id, Ice.Callback __cb)
    {
        return begin_getState(id, null, false, __cb);
    }

    public Ice.AsyncResult begin_getState(String id, java.util.Map<String, String> __ctx, Ice.Callback __cb)
    {
        return begin_getState(id, __ctx, true, __cb);
    }

    public Ice.AsyncResult begin_getState(String id, Callback_VicodeDebug_getState __cb)
    {
        return begin_getState(id, null, false, __cb);
    }

    public Ice.AsyncResult begin_getState(String id, java.util.Map<String, String> __ctx, Callback_VicodeDebug_getState __cb)
    {
        return begin_getState(id, __ctx, true, __cb);
    }

    private Ice.AsyncResult begin_getState(String id, java.util.Map<String, String> __ctx, boolean __explicitCtx, IceInternal.CallbackBase __cb)
    {
        __checkAsyncTwowayOnly(__getState_name);
        IceInternal.OutgoingAsync __result = new IceInternal.OutgoingAsync(this, __getState_name, __cb);
        try
        {
            __result.__prepare(__getState_name, Ice.OperationMode.Idempotent, __ctx, __explicitCtx);
            IceInternal.BasicStream __os = __result.__startWriteParams(Ice.FormatType.DefaultFormat);
            __os.writeString(id);
            __result.__endWriteParams();
            __result.__send(true);
        }
        catch(Ice.LocalException __ex)
        {
            __result.__exceptionAsync(__ex);
        }
        return __result;
    }

    public int end_getState(Ice.AsyncResult __result)
    {
        Ice.AsyncResult.__check(__result, this, __getState_name);
        boolean __ok = __result.__wait();
        try
        {
            if(!__ok)
            {
                try
                {
                    __result.__throwUserException();
                }
                catch(Ice.UserException __ex)
                {
                    throw new Ice.UnknownUserException(__ex.ice_name(), __ex);
                }
            }
            IceInternal.BasicStream __is = __result.__startReadParams();
            int __ret;
            __ret = __is.readInt();
            __result.__endReadParams();
            return __ret;
        }
        catch(Ice.LocalException ex)
        {
            Ice.Instrumentation.InvocationObserver __obsv = __result.__getObserver();
            if(__obsv != null)
            {
                __obsv.failed(ex.ice_name());
            }
            throw ex;
        }
    }

    public static VicodeDebugPrx checkedCast(Ice.ObjectPrx __obj)
    {
        VicodeDebugPrx __d = null;
        if(__obj != null)
        {
            if(__obj instanceof VicodeDebugPrx)
            {
                __d = (VicodeDebugPrx)__obj;
            }
            else
            {
                if(__obj.ice_isA(ice_staticId()))
                {
                    VicodeDebugPrxHelper __h = new VicodeDebugPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static VicodeDebugPrx checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        VicodeDebugPrx __d = null;
        if(__obj != null)
        {
            if(__obj instanceof VicodeDebugPrx)
            {
                __d = (VicodeDebugPrx)__obj;
            }
            else
            {
                if(__obj.ice_isA(ice_staticId(), __ctx))
                {
                    VicodeDebugPrxHelper __h = new VicodeDebugPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static VicodeDebugPrx checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        VicodeDebugPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA(ice_staticId()))
                {
                    VicodeDebugPrxHelper __h = new VicodeDebugPrxHelper();
                    __h.__copyFrom(__bb);
                    __d = __h;
                }
            }
            catch(Ice.FacetNotExistException ex)
            {
            }
        }
        return __d;
    }

    public static VicodeDebugPrx checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        VicodeDebugPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA(ice_staticId(), __ctx))
                {
                    VicodeDebugPrxHelper __h = new VicodeDebugPrxHelper();
                    __h.__copyFrom(__bb);
                    __d = __h;
                }
            }
            catch(Ice.FacetNotExistException ex)
            {
            }
        }
        return __d;
    }

    public static VicodeDebugPrx uncheckedCast(Ice.ObjectPrx __obj)
    {
        VicodeDebugPrx __d = null;
        if(__obj != null)
        {
            if(__obj instanceof VicodeDebugPrx)
            {
                __d = (VicodeDebugPrx)__obj;
            }
            else
            {
                VicodeDebugPrxHelper __h = new VicodeDebugPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static VicodeDebugPrx uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        VicodeDebugPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            VicodeDebugPrxHelper __h = new VicodeDebugPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    public static final String[] __ids =
    {
        "::Ice::Object",
        "::bicacomms::VicodeDebug"
    };

    public static String ice_staticId()
    {
        return __ids[1];
    }

    protected Ice._ObjectDelM __createDelegateM()
    {
        return new _VicodeDebugDelM();
    }

    protected Ice._ObjectDelD __createDelegateD()
    {
        return new _VicodeDebugDelD();
    }

    public static void __write(IceInternal.BasicStream __os, VicodeDebugPrx v)
    {
        __os.writeProxy(v);
    }

    public static VicodeDebugPrx __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            VicodeDebugPrxHelper result = new VicodeDebugPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }

    public static final long serialVersionUID = 0L;
}
