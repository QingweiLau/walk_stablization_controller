#ifndef READ_WALK_PATTERN_HPP
#define READ_WALK_PATTERN_HPP

#include "common.hpp"

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <boost/tokenizer.hpp>

const std::string FileName( "/home/qingweidesk/thorman/ThormanG3Controller/WalkStablizationController/sample.txt" );
//const std::string FileName( "sample.txt" );

const int SKIP_ROWS_NUM = 0;
const int SKIP_COLUMNS_NUM = 1;
const int COLUMNS_NUM = 13;

int readWalkPattern( DoubleArrayVector &avec )
{
	boost::char_separator< char > sep( ",", "", boost::keep_empty_tokens );
	std::ifstream ifs( FileName.c_str() );
	std::string line;

	for ( int i = 0; i < SKIP_ROWS_NUM; ++i )
	{
		getline( ifs, line );
	}
	//std::cout << std::filesystem::current_path() << std::endl;
	//char buf[255];
	//getcwd(buf,sizeof(buf));
	//std::cout << buf << std::endl;
	if (ifs.good()==0)
	{
		std::cout<<"Error encounted while reading data from file."<<std::endl;
		
	}
	
	while ( ifs.good() )
	{
		getline( ifs, line );
		if( line.empty() )
		{
			break;
		}

		typedef boost::tokenizer< boost::char_separator< char > > tokenizer;
		tokenizer tokens( line, sep );

		boost::shared_array< double > data( new double[ COLUMNS_NUM ] );
		tokenizer::iterator it = tokens.begin();

		for ( int i = 0; i < SKIP_COLUMNS_NUM; ++i, ++it );

		for ( int i = 0; i < COLUMNS_NUM; ++i, ++it )
		{
			std::stringstream ss;
			double d;
			ss << *it;
			ss >> d;
			data[ i ] = d;
		/*
		//test senario for delta_x=0;
  			data[0] = 0;
  			data[3] = 0;
 			data[6] = 0;
			data[9] = 0;
			
		*/
		}
		avec.push_back( data );
		//std::cout << "avec.size >>> " << avec.size() << std::endl;

	}
	//std::cout << "last: avec.size >>> " << avec.size() << std::endl;
	return avec.size();
}

#endif
