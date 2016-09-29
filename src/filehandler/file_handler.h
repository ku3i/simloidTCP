#ifndef _FILEHANDLER_H_
#define _FILEHANDLER_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>

class FileHandler
{
	public:
		enum Mode {
			READONLY,			//"r"
			WRITE,				//"w"
			REWRITE = WRITE,	
			APPEND,				//"a"
			READWRITE,			//"r+"
			NEWREADWRITE,		//"w+"
			READAPPEND			//"a+"
		};
		
		FileHandler() {
			errorcode = "";
			filename = "";
			lineNumber = -1;
			file = NULL;
			buffer = NULL;
		};
		~FileHandler() {
			if(file != NULL) fclose(file);
		};
		
		bool setFile(const char* fname, const Mode mode);
		inline const std::string& getFileName() const;
		inline const char* getErrorCode() const;
		
		const char* readNextLine();
		inline void fhRewind();
		inline bool reload();
		inline int getLineNumber() const { return lineNumber; };
		int append(const char* data) const;
		int append(const std::string str) const;
		int append(const int i) const;
		int append(const uint i) const;
		int append(const long l) const;
		int append(const double d) const;
		int append(const char c) const;
		inline int flush() const;
			
	private:
		FILE* file;
		Mode theMode;
		std::string filename;
		char* buffer;
		long  bufferSize;
		std::string errorcode;	//hier wird der Fehler reingeschrieben, falls was passiert
		int lineNumber;
};

inline const char* FileHandler::getErrorCode() const
{
	return errorcode.c_str();
}

inline void FileHandler::fhRewind()
{
	lineNumber = 0;
	//std::rewind(file);
  rewind(file);
}

inline const std::string& FileHandler::getFileName() const
{
	return filename;
}

inline bool FileHandler::reload()
{
	return setFile(filename.c_str(), theMode);
}

inline int FileHandler::flush() const
{
	return fflush(file);
}

#endif
