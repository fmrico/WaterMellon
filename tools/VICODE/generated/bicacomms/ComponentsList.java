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

public class ComponentsList implements java.lang.Cloneable, java.io.Serializable
{
    public String[] ListComps;

    public int numCompos;

    public ComponentsList()
    {
    }

    public ComponentsList(String[] ListComps, int numCompos)
    {
        this.ListComps = ListComps;
        this.numCompos = numCompos;
    }

    public boolean
    equals(java.lang.Object rhs)
    {
        if(this == rhs)
        {
            return true;
        }
        ComponentsList _r = null;
        if(rhs instanceof ComponentsList)
        {
            _r = (ComponentsList)rhs;
        }

        if(_r != null)
        {
            if(!java.util.Arrays.equals(ListComps, _r.ListComps))
            {
                return false;
            }
            if(numCompos != _r.numCompos)
            {
                return false;
            }

            return true;
        }

        return false;
    }

    public int
    hashCode()
    {
        int __h = 5381;
        __h = IceInternal.HashUtil.hashAdd(__h, "::bicacomms::ComponentsList");
        __h = IceInternal.HashUtil.hashAdd(__h, ListComps);
        __h = IceInternal.HashUtil.hashAdd(__h, numCompos);
        return __h;
    }

    public java.lang.Object
    clone()
    {
        java.lang.Object o = null;
        try
        {
            o = super.clone();
        }
        catch(CloneNotSupportedException ex)
        {
            assert false; // impossible
        }
        return o;
    }

    public void
    __write(IceInternal.BasicStream __os)
    {
        ListStringHelper.write(__os, ListComps);
        __os.writeInt(numCompos);
    }

    public void
    __read(IceInternal.BasicStream __is)
    {
        ListComps = ListStringHelper.read(__is);
        numCompos = __is.readInt();
    }

    public static final long serialVersionUID = -2654578006682759912L;
}
