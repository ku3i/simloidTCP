#include "./file_handler.h"

const char* FileHandler::readNextLine()
{
  bool isDataAvailable = false;
  do {
    if ( fgets( buffer, bufferSize, file ) == 0) {
      return NULL;
    }
	++lineNumber;
    /* Kommentare und Leerzeilen ignorieren */
    if (buffer[0] == '#' || buffer[0] == '\n') {
      continue;
    }
    isDataAvailable = true;
  } while ( isDataAvailable == false );
  return buffer;
}

bool FileHandler::setFile(const char* fname, const Mode mode)
{
	//alte Datei ggf. schliessen
	if(file != NULL) fclose(file);
	filename = "";
	if(buffer != NULL) free(buffer);
	lineNumber = -1;

	//neue Datei oeffnen
	switch(mode) {
		case READONLY:
			file = fopen(fname, "r");
			break;

		case WRITE:
			file = fopen(fname, "w");
			break;

		case APPEND:
			file = fopen(fname, "a");
			break;

		case READWRITE:
			file = fopen(fname, "r+");
			break;

		case NEWREADWRITE:
			file = fopen(fname, "w+");
			break;

		case READAPPEND:
			file = fopen(fname, "a+");
			break;

		default:
			errorcode = "Unknown fopen-mode.";
			return false;
	}

	if(file != NULL) {
		// obtain file size.
		fseek (file , 0 , SEEK_END);
  		bufferSize = ftell (file);
  		//std::rewind (file);
      rewind (file);
		lineNumber = 0;

		buffer = (char*) malloc (bufferSize);
		if(buffer == NULL) {
			errorcode = "Could not allocate memory for parsing file";
			return false;
		}
		else {
			//Filename setzen
			filename = fname;
			return true;
		}
	}
	else {
		errorcode = "Could not open file '";
		errorcode += fname;
		errorcode += "'";
		return false;
	}
}

int FileHandler::append(const char* data) const
{
	//if(std::fwrite (data, strlen(data), 1, file) == 1) {
  if(fwrite (data, strlen(data), 1, file) == 1) {
		return strlen(data);
	}
	else {
		return false;
	}
}

int FileHandler::append(const std::string str) const
{
	//if(std::fwrite (str.c_str(), str.size(), 1, file) == 1) {
  if(fwrite (str.c_str(), str.size(), 1, file) == 1) {
		return str.size();
	}
	else {
		return 0;
	}
}

int FileHandler::append(const int i) const
{
	char tmp[1024];
	sprintf(tmp, "%d", i);
	//if(std::fwrite (tmp, strlen(tmp), 1, file) == 1) {
  if(fwrite (tmp, strlen(tmp), 1, file) == 1) {
		return strlen(tmp);
	}
	else {
		return 0;
	}
}

int FileHandler::append(const uint i) const
{
	char tmp[1024];
	sprintf(tmp, "%d", i);
	//if(std::fwrite (tmp, strlen(tmp), 1, file) == 1) {
  if(fwrite (tmp, strlen(tmp), 1, file) == 1) {
		return strlen(tmp);
	}
	else {
		return 0;
	}
}

int FileHandler::append(const long l) const
{
	char tmp[1024];
	sprintf(tmp, "%ld", l);
	//if(std::fwrite (tmp, strlen(tmp), 1, file) == 1) {
  if(fwrite (tmp, strlen(tmp), 1, file) == 1) {
		return strlen(tmp);
	}
	else {
		return 0;
	}
}

int FileHandler::append(const double d) const
{
	char tmp[1024];
	sprintf(tmp, "%lf", d);
	//if(std::fwrite (tmp, strlen(tmp), 1, file) == 1) {
  if(fwrite (tmp, strlen(tmp), 1, file) == 1) {
		return strlen(tmp);
	}
	else {
		return 0;
	}
}

int FileHandler::append(const char c) const
{
	//if(std::fputc(c, file) == c) {
  if(fputc(c, file) == c) {
		return 1;
	}
	else {
		return 0;
	}
}
