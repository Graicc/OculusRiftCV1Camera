/* SAL annotation shim so the vendored DirectShow base classes (streams.h/cpp)
   compile under MinGW-w64, which does not define the double-underscore SAL
   macros used by MSVC headers. All annotations are documentation-only. */
#ifndef MINGW_SAL_SHIM_H
#define MINGW_SAL_SHIM_H

#ifndef _MSC_VER

#define __in
#define __in_opt
#define __in_bcount(x)
#define __in_bcount_opt(x)
#define __in_ecount(x)
#define __in_ecount_opt(x)
#define __out
#define __out_opt
#define __out_bcount(x)
#define __out_ecount(x)
#define __out_ecount_part(x,y)
#define __inout
#define __inout_opt
#define __inout_ecount_full(x)
#define __deref_in
#define __deref_inout_opt
#define __deref_out
#define __deref_out_opt
#define __field_ecount_opt(x)
#define __format_string
#ifndef __inline
#define __inline inline
#endif

#endif

#endif
