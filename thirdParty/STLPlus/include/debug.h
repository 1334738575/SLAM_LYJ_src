#ifndef STLPLUS_DEBUG
#define STLPLUS_DEBUG
////////////////////////////////////////////////////////////////////////////////

//   Author:    Andy Rushton
//   Copyright: (c) Southampton University 1999-2004
//              (c) Andy Rushton           2004 onwards
//   License:   BSD License, see ../docs/license.html

//   Set of simple debug utilities, all of which are switched off by the
//   NDEBUG compiler directive

////////////////////////////////////////////////////////////////////////////////

#include "portability_fixes.h"
#include <stdexcept>
#include <string>

////////////////////////////////////////////////////////////////////////////////
// Problem with missing __FUNCTION__ macro
////////////////////////////////////////////////////////////////////////////////

// this macro is used in debugging but was missing in Visual Studio prior to version 7
#if defined(_MSC_VER) && (_MSC_VER < 1300)
#define __FUNCTION__ 0
#endif

// old versions of Borland compiler defined a macro __FUNC__ but more recent ones define __FUNCTION__
// I'm not sure at what version this change was made - assumed C++ Builder 6.32
#if defined(__BORLANDC__) && (__BORLANDC__ < 1585)
#define __FUNCTION__ __FUNC__
#endif

////////////////////////////////////////////////////////////////////////////////
// Exception thrown if an assertion fails

namespace stlplus
{

  class assert_failed : public std::logic_error
  {
  public:
    assert_failed(const char* file, int line, const char* function, const char* message) ;
    ~assert_failed(void) throw();
  };

} // end namespace stlplus

  ////////////////////////////////////////////////////////////////////////////////
  // The macros used in debugging

#ifndef NDEBUG

#define DEBUG_TRACE stlplus::debug_trace stlplus_debug_trace(__FILE__,__LINE__,__FUNCTION__)
#define IF_DEBUG(stmts) {if (stlplus_debug_trace.debug()){stlplus_debug_trace.prefix(__LINE__);stmts;}}
#define DEBUG_REPORT(str) IF_DEBUG(stlplus_debug_trace.report(__LINE__,str))
#define DEBUG_ERROR(str) stlplus_debug_trace.error(__LINE__,str)
#define DEBUG_STACKDUMP(str) stlplus_debug_trace.stackdump(__LINE__,str)
#define DEBUG_ON stlplus_debug_trace.debug_on(__LINE__,true)
#define DEBUG_ON_LOCAL stlplus_debug_trace.debug_on(__LINE__,false)
#define DEBUG_ON_GLOBAL stlplus::debug_global(__FILE__,__LINE__,__FUNCTION__,true)
#define DEBUG_OFF_GLOBAL stlplus::debug_global(__FILE__,__LINE__,__FUNCTION__,false)
#define DEBUG_OFF stlplus_debug_trace.debug_off(__LINE__)
#define DEBUG_ASSERT(test) if (!(test))stlplus::debug_assert_fail(__FILE__,__LINE__,__FUNCTION__,#test)

#else

#define DEBUG_TRACE
#define IF_DEBUG(stmts)
#define DEBUG_REPORT(str)
#define DEBUG_ERROR(str)
#define DEBUG_STACKDUMP(str)
#define DEBUG_ON
#define DEBUG_ON_LOCAL
#define DEBUG_ON_GLOBAL
#define DEBUG_OFF_GLOBAL
#define DEBUG_OFF
#define DEBUG_ASSERT(test)

#endif

////////////////////////////////////////////////////////////////////////////////
// infrastructure - don't use directly

namespace stlplus
{

  void debug_global(const char* file, int line, const char* function, bool state = true);
  // exceptions: assert_failed
  void debug_assert_fail(const char* file, int line, const char* function, const char* test) ;

  class debug_trace
  {
  public:
    debug_trace(const char* f, int l, const char* fn);
    ~debug_trace(void);
    const char* file(void) const;
    int line(void) const;
    bool debug(void) const;
    void debug_on(int l, bool recurse);
    void debug_off(int l);
    void prefix(int l) const;
    void report(int l, const std::string& message) const;
    void report(const std::string& message) const;
    void error(int l, const std::string& message) const;
    void error(const std::string& message) const;
    void stackdump(int l, const std::string& message) const;
    void stackdump(const std::string& message) const;
    void stackdump(void) const;

  private:
    const char* m_file;
    int m_line;
    const char* m_function;
    unsigned m_depth;
    const debug_trace* m_last;
    bool m_dbg;
    bool m_old;
    void do_report(int l, const std::string& message) const;
    void do_report(const std::string& message) const;

    // make this class uncopyable
    debug_trace(const debug_trace&);
    debug_trace& operator = (const debug_trace&);
  };

} // end namespace stlplus

  ////////////////////////////////////////////////////////////////////////////////
#endif
