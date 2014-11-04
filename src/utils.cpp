/**
 * @file
 * C++ helper functions.
 *
 * Copyright (C) 2014 Manuel García (Manu@facine.es)
 */

#include "utils.hpp"

/**
 * File helper functions.
 */

const char *easyUtils::findPatternInPath (const char *path, const char *pattern) {
  DIR *dir = opendir (path);

  if (dir) {
    struct dirent *ent;
    while((ent = readdir (dir)) != NULL) {
      if (strncmp(ent->d_name, pattern, strlen (pattern)) == 0) {
        closedir (dir);
        return ent->d_name;
      }
    }
    closedir (dir);
    return "";
  }
  else {
    std::cout << "Error opening the directory: " << std::string (path) << "!" << std::endl;
    exit (EXIT_FAILURE);
  }
}

const std::string easyUtils::findPatternInPath (const std::string &path, const std::string &pattern) {
  return findPatternInPath (path.c_str (), pattern.c_str ());
}

unsigned char easyUtils::fileExists (const char *fileName) {
  std::ifstream textFile (fileName);

  if (textFile.good ()) {
    textFile.close ();
    return true;
  }
  else {
    return false;
  }
}

unsigned char easyUtils::fileExists (const std::string &fileName) {
  return fileExists (fileName.c_str ());
}

std::string easyUtils::readTextFile (const char *fileName) {
  std::ifstream textFile (fileName);
  std::stringstream buffer;

  buffer << textFile.rdbuf ();

  return buffer.str ();
}

std::string easyUtils::readTextFile (const std::string &fileName) {
  std::ifstream textFile (fileName);
  std::stringstream buffer;

  buffer << textFile.rdbuf ();

  return buffer.str ();
}

void easyUtils::writeTextFile (const char *fileName, const char *text) {
  std::ofstream textFile (fileName);

  if (textFile.is_open ()) {
    textFile << text;
    textFile.close ();
  }
  else {
    std::cout << "Unable to open the file: " << fileName << "!" << std::endl;
    exit (EXIT_FAILURE);
  }
}

void easyUtils::writeTextFile (const std::string &fileName, const std::string &text) {
  std::ofstream textFile (fileName);

  if (textFile.is_open ()) {
    textFile << text;
    textFile.close ();
  }
  else {
    std::cout << "Unable to open the file: " << fileName << "!" << std::endl;
    exit (EXIT_FAILURE);
  }
}

/**
 * Strings helper functions.
 */

std::string easyUtils::intToHex (int i) {
  std::stringstream stream;

  stream << std::hex << i;
//  << std::setfill ('0') << std::setw(sizeof(T)*2)

  return stream.str();
}

unsigned char easyUtils::str_match (const std::string &search, const std::string &subject) {
  size_t start_pos = subject.find (search);

  if (start_pos == std::string::npos) {
    return false;
  }

  return true;
}

unsigned char easyUtils::str_match (const char *search, const std::string &subject) {
  std::string str_search (search);

  return str_match (str_search, subject);
}

unsigned char easyUtils::str_replace (std::string &subject, const std::string &search, const std::string &replace) {
  size_t start_pos = subject.find (search);

  if (start_pos == std::string::npos) {
    return false;
  }

  subject.replace (start_pos, search.length (), replace);

  return true;
}

unsigned char easyUtils::str_replace_all (std::string &subject, const std::string &search, const std::string &replace) {
  size_t start_pos = subject.find (search);

  if (search.empty () || start_pos == std::string::npos) {
    return false;
  }

  start_pos = 0;

  while ((start_pos = subject.find (search, start_pos)) != std::string::npos) {
    subject.replace(start_pos, search.length (), replace);
     // In case 'search' contains 'replace', like replacing 'x' with 'yx'.
    start_pos += replace.length ();
  }

  return true;
}

std::string easyUtils::str_tolower (const std::string &upperCase) {
  std::string lowerCase;

  std::transform (upperCase.begin (), upperCase.end (), std::back_inserter (lowerCase), ::tolower);

  return lowerCase;
}

std::string easyUtils::str_toupper (const std::string &lowerCase) {
  std::string upperCase;

  std::transform (lowerCase.begin (), lowerCase.end (), std::back_inserter (upperCase), ::toupper);

  return upperCase;
}

std::string &easyUtils::trim (std::string &text) {
  return trim_left (trim_right (text));
}

std::string &easyUtils::trim_left (std::string &text) {
  text.erase (text.begin (), std::find_if (text.begin (), text.end (), std::not1 (std::ptr_fun <int, int> (std::isspace))));

  return text;
}

std::string &easyUtils::trim_right (std::string &text) {
  text.erase (std::find_if (text.rbegin (), text.rend (), std::not1 (std::ptr_fun <int, int> (std::isspace))).base (), text.end ());

  return text;
}
