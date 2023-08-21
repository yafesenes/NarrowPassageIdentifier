#ifndef PGMREADER_CLASS_H
#define PGMREADER_CLASS_H
#include <iostream>
#include <fstream>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <string>
#include <sstream>

using namespace std;

class pgmreader
{
public:
    static vector<vector<int>> readMap(const string& workingdir, const string& yamlfile)
    {
        try {
            YAML::Node map = YAML::LoadFile(workingdir + yamlfile);
            std::string pgmfile = map["image"].as<std::string>();
            double resolution = map["resolution"].as<double>();
            std::vector<double> origin = map["origin"].as<std::vector<double>>();
            int negate = map["negate"].as<int>();
            double occupied_thresh = map["occupied_thresh"].as<double>();
            double free_thresh = map["free_thresh"].as<double>();

            ifstream file;
            file.open(workingdir + pgmfile);

            if (!file)
            {
                cerr << "File could not be opened!" << endl;
                return vector<vector<int>>();
            }

            string magicNumber;
            getline(file, magicNumber);
            if (magicNumber != "P5")
            {
                cerr << "Wrong pgm file format: " << magicNumber << endl;
                return vector<vector<int>>();
            }
            string line;
            getline(file, line);
            int width, height, maxVal;
            if (line.size() > 0 && line[0] == '#')
                file >> width >> height >> maxVal;
            else
            {
                istringstream iss(line);
                iss >> width >> height >> maxVal;
            }

            //file.ignore(256, '\n');

            vector<vector<int>> pixels(height, vector<int>(width));
            for (int i = 0; i < height; i++)
            {
                for (int j = 0; j < width; j++)
                {
                    float p = (maxVal - file.get()) / (float)maxVal;
                    if (p > occupied_thresh)
                        pixels[i][j] = 1;
                    else if (p < free_thresh)
                        pixels[i][j] = 0;
                    else
                        pixels[i][j] = -1;
                }
            }
            file.close();

            return pixels;
        }
        catch (const YAML::BadFile& e)
        {
            cout << "YAML badFile " << workingdir+yamlfile << endl;
        }
        catch (const YAML::ParserException& e)
        {
            cout << "YAML parser exception" << endl;
        }
        catch (const YAML::Exception& e)
        {
            cout << "exception " << endl;
        }
        return std::vector<std::vector<int>>();
    }
};

#endif
