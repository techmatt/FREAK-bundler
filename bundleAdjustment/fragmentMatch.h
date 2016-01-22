
/*struct Correspondence
{
    struct Entry
    {
        int fragmentIndex;
        vec3f fragmentPos;
    };
    Entry e[2];
    float weight;
    float alignmentError;
    float scoreAtoB;
    float scoreBtoA;
};*/

/*struct FragmentMatch
{
    void loadASCII(const string &filename, int _fragmentAIndex, int _fragmentBIndex)
    {
        fragmentAIndex = _fragmentAIndex;
        fragmentBIndex = _fragmentBIndex;

        auto lines = util::getFileLines(filename);
        
        /*
        matchCount 34
        keypointsACount 307
        keypointsBCount 190
        transform
        0.846259 -0.161336 0.507756 0.399105
        0.0182474 0.961264 0.275023 -0.42565
        -0.532459 -0.223476 0.816423 1.71167
        0 0 0 1
        #matches*/

        /*const int matchCount = convert::toInt(util::split(lines[0], ' ')[1]);
        const int keypointACount = convert::toInt(util::split(lines[1], ' ')[1]);
        const int keypointBCount = convert::toInt(util::split(lines[2], ' ')[1]);

        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                transformBtoA(i, j) = convert::toFloat(util::split(lines[i + 4], ' ')[j]);
            }
        }
        //transformBtoA = transformBtoA.getInverse();

        const int matchStart = 9;
        const int keypointAStart = matchStart + matchCount + 1;
        const int keypointBStart = keypointAStart + keypointACount + 1;

        for (int matchIndex = 0; matchIndex < matchCount; matchIndex++)
        {
            const auto parts = util::split(lines[matchStart + matchIndex], ' ');
            assert(parts.size() == 9);
            //1.32859 1.56203 1.10664 0.953906 1.97172 0.646846 0.000261162
            Correspondence c;
            c.e[0].fragmentIndex = _fragmentAIndex;
            c.e[0].fragmentPos[0] = convert::toFloat(parts[0]);
            c.e[0].fragmentPos[1] = convert::toFloat(parts[1]);
            c.e[0].fragmentPos[2] = convert::toFloat(parts[2]);

            c.e[1].fragmentIndex = _fragmentBIndex;
            c.e[1].fragmentPos[0] = convert::toFloat(parts[3]);
            c.e[1].fragmentPos[1] = convert::toFloat(parts[4]);
            c.e[1].fragmentPos[2] = convert::toFloat(parts[5]);

            // this could be proportional to the distance
            c.alignmentError = convert::toFloat(parts[6]);
            c.scoreAtoB = convert::toFloat(parts[7]);
            c.scoreBtoA = convert::toFloat(parts[8]);
            c.weight = 1.0f;
            correspondences.push_back(c);
        }

        for (int keypointAIndex = 0; keypointAIndex < keypointACount; keypointAIndex++)
        {
            const auto parts = util::split(lines[keypointAStart + keypointAIndex], ' ');
            vec3f v;
            v[0] = convert::toFloat(parts[0]);
            v[1] = convert::toFloat(parts[1]);
            v[2] = convert::toFloat(parts[2]);
            keypointsA.push_back(v);
        }
        for (int keypointBIndex = 0; keypointBIndex < keypointBCount; keypointBIndex++)
        {
            const auto parts = util::split(lines[keypointBStart + keypointBIndex], ' ');
            vec3f v;
            v[0] = convert::toFloat(parts[0]);
            v[1] = convert::toFloat(parts[1]);
            v[2] = convert::toFloat(parts[2]);
            keypointsA.push_back(v);
        }
    }

    int strongCorrespondences(float threshold) const
    {
        int result = 0;
        for (auto &c : correspondences)
            if (min(c.scoreAtoB, c.scoreBtoA) >= threshold)
                result++;
        return result;
    }
    float strongCorrespondenceRatio(float threshold) const
    {
        if (correspondences.size() == 0)
            return 0.0f;
        return (float)strongCorrespondences(threshold) / correspondences.size();
    }

    mat4f transformBtoA; // not used directly by solver
    int fragmentAIndex;
    int fragmentBIndex;

    vector<Correspondence> correspondences;
    vector<vec3f> keypointsA;
    vector<vec3f> keypointsB;
};*/
