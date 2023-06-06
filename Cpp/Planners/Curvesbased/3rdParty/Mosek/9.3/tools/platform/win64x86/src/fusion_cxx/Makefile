
CXX=cl.exe
LD=link.exe
MSKMAJORVER=9
MSKMINORVER=3
LIBTOOL=lib.exe

CFLAGS=/O2 /EHsc
IPATHS=/I..\..\h /I.

fusion64_$(MSKMAJORVER)_$(MSKMINORVER).lib: fusion.obj BaseModel.obj Debug.obj IntMap.obj SolverInfo.obj StringBuffer.obj mosektask.obj
	$(LIBTOOL) /out:fusion64_$(MSKMAJORVER)_$(MSKMINORVER).lib fusion.obj BaseModel.obj Debug.obj IntMap.obj SolverInfo.obj StringBuffer.obj mosektask.obj

.cc.obj:
	$(CXX) $(IPATHS) $(CFLAGS) /c $<

install: ..\..\bin\fusion64_$(MSKMAJORVER)_$(MSKMINORVER).lib

..\..\bin\fusion64_$(MSKMAJORVER)_$(MSKMINORVER).lib: fusion64_$(MSKMAJORVER)_$(MSKMINORVER).lib
	copy fusion64_$(MSKMAJORVER)_$(MSKMINORVER).lib ..\..\bin
clean:
	del *.obj *.dll *.lib
	del ..\..\bin\fusion64_$(MSKMAJORVER)_$(MSKMINORVER).lib