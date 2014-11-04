/**
 * @file
 * Headers file of C++ helper functions.
 *
 * Copyright (C) 2014 Manuel García (Manu@facine.es)
 */

#ifndef _UTILS_HPP_
#define _UTILS_HPP_

#include <algorithm>
#include <cctype>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <functional>
#include <locale>
#include <string>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <sys/types.h>

namespace easyUtils {
  /**
   * File helper functions.
   */

  /**
   * Find pattern in path.
   *
   * @param const char *path
   *   The path where we have to look for.
   * @param const char *pattern
   *   The pattern to look for.
   *
   * @return const char *
   *   The first file/directory or empty value.
   */
  const char *findPatternInPath (const char *path, const char *pattern);

  /**
   * Find pattern in path.
   *
   * @param const char *path
   *   The path where we have to look for.
   * @param const char *pattern
   *   The pattern to look for.
   *
   * @return const const std::string
   *   The first file/directory or empty value.
   */
  const std::string findPatternInPath (const std::string &path, const std::string &pattern);

  /**
   * Check if file exists.
   *
   * @param const char *fileName
   *   The path and name of the file.
   *
   * @return unsigned char
   *   Can return either TRUE or FALSE values.
   */
  unsigned char fileExists (const char *fileName);

  /**
   * Check if file exists.
   *
   * @param const std::string &fileName
   *   The path and name of the file.
   *
   * @return unsigned char
   *   Can return either TRUE or FALSE values.
   */
  unsigned char fileExists (const std::string &fileName);

  /**
   * Read text file.
   *
   * @param const char *fileName
   *   The path and name of the file.
   *
   * @return std::string
   *   Return the text of file.
   */
  std::string readTextFile (const char *fileName);

  /**
   * Read text file.
   *
   * @param const std::string &fileName
   *   The path and name of the file.
   *
   * @return std::string
   *   Return the text of file.
   */
  std::string readTextFile (const std::string &fileName);

  /**
   * Write a text file.
   *
   * @param const char *fileName
   *   The path and name of the file.
   * @param const char *text
   *   The text to write.
   */
  void writeTextFile (const char *fileName, const char *text);

  /**
   * Write a text file.
   *
   * @param const std::string &fileName
   *   The path and name of the file.
   * @param const std::string &text
   *   The text to write.
   */
  void writeTextFile (const std::string &fileName, const std::string &text);

  /**
   * String helper functions.
   */

  /**
   * Convert integer to hexadecimal.
   *
   * @param int i
   *   The value to convert.
   *
   * @return std::string
   *   Strig value in hexadecimal format.
   */
  std::string intToHex (int i);

  /**
   * Search at least one occurrence of the search string.
   *
   * @param const std::string &search
   *   The value being searched for.
   * @param const std::string &subject
   *   The string being searched.
   *
   * @return unsigned char
   *   Return true of false, if find at least one.
   */
  unsigned char str_match (const std::string &search, const std::string &subject);

  /**
   * Search at least one occurrence of the search string.
   *
   * @param const std::string *search
   *   The value being searched for.
   * @param const std::string &subject
   *   The string being searched.
   *
   * @return unsigned char
   *   Return true of false, if find at least one.
   */
  unsigned char str_match (const char *search, const std::string &subject);

  /**
   * Replace fisrt occurrence of the search string with the replacement string.
   *
   * @param std::string &subject
   *   The string being searched and replaced on, otherwise known as the
   *   haystack.
   * @param std::string &search
   *   The value being searched for, otherwise known as the needle.
   * @param std::string &replace
   *   The replacement value that replaces found search values.
   *
   * @return unsigned char
   *   Return true of false, if find and replace at least one.
   */
  unsigned char str_replace(std::string &subject, const std::string &search, const std::string &replace);

  /**
   * Replace all occurrences of the search string with the replacement string.
   *
   * @param std::string &subject
   *   The string being searched and replaced on, otherwise known as the
   *   haystack.
   * @param std::string &search
   *   The value being searched for, otherwise known as the needle.
   * @param std::string &replace
   *   The replacement value that replaces found search values.
   *
   * @return unsigned char
   *   Return true of false, if find and replace at least one.
   */
  unsigned char str_replace_all(std::string &subject, const std::string &search, const std::string &replace);

  /**
   * Convert string to lower case.
   *
   * @param const std::string &replace
   *   The string to convert.
   *
   * @return ustd::string
   *   The converted string.
   */
  std::string str_tolower (const std::string &upperCase);
  
  /**
   * Convert string to upper case.
   *
   * @param const std::string &replace
   *   The string to convert.
   *
   * @return ustd::string
   *   The converted string.
   */
  std::string str_toupper (const std::string &lowerCase);

  /**
   * Strip whitespace from the beginning and end of a string.
   *
   * @param const std::string &text
   *   The string to strip.
   */
  std::string &trim (std::string &text);

  /**
   * Strip whitespace from the beginning of a string.
   *
   * @param const std::string &text
   *   The string to strip.
   */
  std::string &trim_left (std::string &text);

  /**
   * Strip whitespace from the end of a string.
   *
   * @param const std::string &text
   *   The string to strip.
   */
  std::string &trim_right (std::string &text);
}
#endif
