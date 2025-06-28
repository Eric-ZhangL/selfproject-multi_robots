#include "gpxdata.h"


TGpxData::TGpxData()
{
    trackcount=0;
};

void TGpxData::SaveTrack(TiXmlElement *xmlElement0, TGpxTrack pgt)
{
    TiXmlElement *xmlElement1, *xmlElement2;
    TiXmlText *xmlText;

    xmlElement1 = new TiXmlElement("name");
    xmlElement0->LinkEndChild(xmlElement1);
    xmlText = new TiXmlText(pgt.name.c_str());
    xmlElement1->LinkEndChild(xmlText);

    xmlElement1 = new TiXmlElement("trkseg");
    xmlElement0->LinkEndChild(xmlElement1);

    for (int i = 0; i < pgt.points.size(); i++)
    {
        xmlElement2 = new TiXmlElement("trkpt");
        xmlElement1->LinkEndChild(xmlElement2);
        char str[100];
        sprintf(str, "%.8f", pgt.points[i].x);
        xmlElement2->SetAttribute("lat", str);
        sprintf(str, "%.8f", pgt.points[i].y);
        xmlElement2->SetAttribute("lon", str);
    }
};

void TGpxData::Save(char *filename)
{
    // printf("999999999999999999999999999999999999999999999999999999999999\n");
    if (trackcount == 0)
        return;

    // printf("%s \n", filename);

    TiXmlDocument xmlDoc;
    TiXmlElement *xmlElement0, *xmlElement1;
    TiXmlText *xmlText;
    TiXmlDeclaration *pDeclaration;

    pDeclaration = new TiXmlDeclaration("1.0", "UTF-8", "");
    xmlDoc.LinkEndChild(pDeclaration);

    xmlElement0 = new TiXmlElement("gpx");
    xmlDoc.LinkEndChild(xmlElement0);
    xmlText = new TiXmlText("version=1.0 Created by VSC"); ///文本
    xmlElement0->LinkEndChild(xmlText);

    xmlElement1 = new TiXmlElement("time");
    xmlElement0->LinkEndChild(xmlElement1);

    time_t timep;
    time(&timep);
    xmlText = new TiXmlText(ctime(&timep)); ///文本
    xmlElement1->LinkEndChild(xmlText);

    for (int i = 0; i < trackcount; i++)
    {
        xmlElement1 = new TiXmlElement("trk");
        xmlElement0->LinkEndChild(xmlElement1);
        SaveTrack(xmlElement1, track[i]);
    }

    xmlDoc.SaveFile(filename);
};

void TGpxData::LoadTrack(TiXmlElement *e0, TGpxTrack *pgt)
{
    printf("999999999999999999999999999999999999999999999999999999999999\n");
    TiXmlElement *e1, *e2;
    TiXmlAttribute *attr1, *attr2;
    geometry_msgs::Point point;

    pgt->Clear();
    e1 = e0->FirstChildElement("name");
    if (e1 != NULL)
    {
        // pgt->name=e1->GetText();
        // printf("%s\n",pgt->name.c_str());
    }
    e1 = e1->NextSiblingElement("trkseg");
    if (e1 != NULL)
    {
        e2 = e1->FirstChildElement("trkpt");
        while (e2 != NULL)
        {
            attr1 = e2->FirstAttribute();
            attr2 = attr1->Next();
            if (strcmp(attr1->Name(), "lat") == 0)
                point.x = attr1->DoubleValue(), point.y = attr2->DoubleValue();
            else if (strcmp(attr1->Name(), "lon") == 0)
                point.x = attr2->DoubleValue(), point.y = attr1->DoubleValue();
            ;
            pgt->Add(point);

            e2 = e2->NextSiblingElement("trkpt");
        }
    }
};

void TGpxData::Load(char *filename)
{
    printf("999999999999999999999999999999999999999999999999999999999999\n\n\n\n\n");
    trackcount = 0;

    TiXmlDocument xmlDoc(filename);
    xmlDoc.LoadFile();
    TiXmlElement *xmlElement0, *xmlElement1, *xmlElement2;

    // printf("%s\n", filename);

    //获取根节点
    xmlElement0 = xmlDoc.RootElement();
    if (xmlElement0 == NULL)
        return;

    xmlElement1 = xmlElement0->FirstChildElement("trk");
    while (xmlElement1 != NULL && trackcount < MaxTrackCount)
    {
        // printf("O");
        LoadTrack(xmlElement1, &track[trackcount]);
        trackcount++;
        xmlElement1 = xmlElement1->NextSiblingElement("trk");
    }

    // Print();
    
};


