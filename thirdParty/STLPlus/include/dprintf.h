#ifndef STLPLUS_DPRINTF
#define STLPLUS_DPRINTF
////////////////////////////////////////////////////////////////////////////////
//
//   Author:    Andy Rushton
//   Copyright: (c) Southampton University 1999-2004
//              (c) Andy Rushton           2004 onwards
//   License:   BSD License, see ../docs/license.html
//
//   Provides an sprintf-like function acting on STL strings. The 'd' in dprintf
//   stands for "dynamic" in that the string is a dynamic string whereas a char*
//   buffer would be static (in size that is, not static in C terms).
//
//   The obvious solution to the problem of in-memory formatted output is to use
//   sprintf(), but this is a potentially dangerous operation since it will quite
//   happily charge off the end of the string it is printing to and thereby
//   corrupt memory. This kind of buffer-overflow vulnerability is the source of
//   most security failures exploited by virus-writers. It means that sprintf
//   should *never* be used and should be made obsolete.
//
//   In any case, using arbitrary-sized fixed-length buffers is not part of any
//   quality-orientated design philosophy.
//
//   Most operating systems now have a safe version of sprintf, but this is
//   non-standard. The functions in this file are platform-independent interfaces
//   to the underlying safe implementation.
//
//   I would like to make this set of functions obsolete too, since I believe the
//   C runtime should be deprecated in favour of C++ runtime which uses dynamic
//   strings and can handle exceptions. However, there is as yet no C++
//   equivalent functionality to some of the string-handling available through
//   the printf-like functions, so it has to stay for now.
//
//     int dprintf (std::string& buffer, const char* format, ...);
//
//       Formats the message by appending to the std::string buffer according to
//       the formatting codes in the format string. The return int is the number
//       of characters generated by this call, i.e. the increase in the length of
//       the std::string.
//
//     int vdprintf (std::string& buffer, const char* format, va_list args);
//
//       As above, but using a pre-initialised va_args argument list. Useful for
//       nesting dprintf calls within variable argument functions.
//
//     std::string dformat (const char* format, ...);
//
//       Similar to dprintf() above, except the result is formatted into a new
//       std::string which is returned by the function. Very useful for inline
//       calls within an iostream expression.
//
//       e.g.    cout << "Total: " << dformat("%6i",t) << endl;
//
//     std::string vdformat (const char* format, va_list);
//
//       As above, but using a pre-initialised va_args argument list. Useful for nesting
//       dformat calls within variable argument functions.
//
//   The format string supports the following format codes as in the C runtime library:
//
//     % [ flags ] [ field ] [ . precision ] [ modifier ] [ conversion ]
//
//     flags:
//       -    - left justified
//       +    - print sign for +ve numbers
//       ' '  - leading space where + sign would be
//       0    - leading zeros to width of field
//       #    - alternate format
//
//     field:
//       a numeric argument specifying the field width - default = 0
//       * means take the next va_arg as the field width - if negative then left justify
//
//     precision:
//       a numeric argument the meaning of which depends on the conversion -
//       - %s - max characters from a string - default = strlen()
//       - %e, %f - decimal places to be displayed - default = 6
//       - %g - significant digits to be displayed - default = 6
//       - all integer conversions - minimum digits to display - default = 0
//       * means take the next va_arg as the field width - if negative then left justify
//
//     modifier:
//       h    - short or unsigned short
//       l    - long or unsigned long
//       L    - long double
//
//     conversions:
//       d, i - short/int/long as decimal
//       u    - short/int/long as unsigned decimal
//       o    - short/int/long as unsigned octal - # adds leading 0
//       x, X - short/int/long as unsigned hexadecimal - # adds leading 0x
//       c    - char
//       s    - char*
//       f    - double/long double as fixed point
//       e, E - double/long double as floating point
//       g, G - double/long double as fixed point/floating point depending on value
//       p    - void* as unsigned hexadecimal
//       %    - literal %
//       n    - int* as recipient of length of formatted string so far
//
////////////////////////////////////////////////////////////////////////////////

#include "portability_fixes.h"
#include <string>
#include <stdexcept>
#include <stdarg.h>

namespace stlplus
{

  // format by appending to a string and return the increase in length
  // if there is an error, return a negative number and leave the string unchanged
  int dprintf (std::string& formatted, const char* format, ...);
  int vdprintf (std::string& formatted, const char* format, va_list args);

  // format into a new string and return the result
  // if there is an error, throw an exception
  // exceptions: std::invalid_argument
  std::string dformat (const char* format, ...) ;
  // exceptions: std::invalid_argument
  std::string vdformat (const char* format, va_list) ;

}

#endif
