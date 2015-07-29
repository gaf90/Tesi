#include "doorlinesegment.h"


namespace SemanticMapping{
namespace Geometry{


DoorLineSegment::DoorLineSegment(const SLAM::Geometry::Point &p1, const SLAM::Geometry::Point &p2):
    LineSegment(p1, p2)
{
    first = NULL;
    second = NULL;
}

DoorLineSegment::DoorLineSegment(double x1, double y1, double x2, double y2):
    LineSegment(x1, y1, x2, y2)
{
    first = NULL;
    second = NULL;
}

SLAM::Geometry::LineSegment DoorLineSegment::getPSegment(double distance)
{
    double m = -1/this->m();
    bool orizontal = false;
    bool vertical = false;
    double m_2, d_2 ,xC, yC, x1, y1, x2, y2;
    //TODO fa un po schifo; come funzionano i NAN e i inf? Gestirli! testarli! OOOO
    if ((this->m() >= 1000) || (this->m() <= -1000 ))
        orizontal = true;
    if ((m >= 1000) || (m <= -1000 ))
        vertical = true;
    if ((!orizontal) && (!vertical))
    {
        //Solve[(y2 - y1) == m (x2 - x1) && (x1 - x2)^2 + (y1 - y2)^2 == d^2 && (x1 + x2)/2 == xC && (y1 + y2)/2 == yC, {x1, y1, x2, y2}] \\FullSimplify
        m_2 = pow(m,2);
        d_2 = pow(distance,2);
        xC = this->centroid().x();
        yC = this->centroid().y();
        x1 = (-sqrt(d_2+d_2*m_2)+2*xC+2*m_2*xC) / (2*(1+m_2));
        y1 = 0.5 * (-m * sqrt( d_2 * ( 1 + m_2 )) / ( 1 + m_2 ) -
                    2*m*xC + (2 * m * xC) / (1 + m_2) +
                    (2 * pow(m,3) * xC ) / ( 1+m_2 ) + 2*yC);
        x2 = 0.5 * ( sqrt( d_2 * ( 1 + m_2 )) / ( 1 + m_2 ) + 4*xC -
                2 * xC / (1 + m_2) - 2 * m_2 * xC / (1 + m_2));
        y2 = m * sqrt(d_2 * (1 + m_2)) * 0.5 / (1 + m_2) +
                m * xC - m * xC / (1 + m_2) - pow(m,3) * xC / (1 + m_2)
                + yC;
    }
    if (orizontal)
    {
        xC = this->centroid().x();
        yC = this->centroid().y();
        x1 = xC + 0.5 * distance;
        y1 = yC;
        x2 = xC - 0.5 * distance;
        y2 = yC;
    }
    if (vertical)
    {
        xC = this->centroid().x();
        yC = this->centroid().y();
        x1 = xC;
        y1 = yC + 0.5 * distance;
        x2 = xC;
        y2 = yC - 0.5 * distance;
    }
    return SLAM::Geometry::LineSegment(x1,y1,x2,y2);
}

SLAM::Geometry::LineSegment DoorLineSegment::getPSegment()
{
    double m = -1/this->m();
    bool orizontal = false;
    bool vertical = false;
    double m_2, d_2 ,xC, yC, x1, y1, x2, y2;
    //TODO fa un po schifo; come funzionano i NAN e i inf? Gestirli! testarli! OOOO
    if ((this->m() >= 1000) || (this->m() <= -1000 ))
        orizontal = true;
    if ((m >= 1000) || (m <= -1000 ))
        vertical = true;
    if ((!orizontal) && (!vertical))
    {
        //Solve[(y2 - y1) == m (x2 - x1) && (x1 - x2)^2 + (y1 - y2)^2 == d^2 && (x1 + x2)/2 == xC && (y1 + y2)/2 == yC, {x1, y1, x2, y2}]
        m_2 = pow(m,2);
        d_2 = pow(DOOR_P_SEGMENT,2);
        xC = this->centroid().x();
        yC = this->centroid().y();
        x1 = (-sqrt(d_2+d_2*m_2)+2*xC+2*m_2*xC) / (2*(1+m_2));
        y1 = 0.5 * (-m * sqrt( d_2 * ( 1 + m_2 )) / ( 1 + m_2 ) -
                    2*m*xC + (2 * m * xC) / (1 + m_2) +
                    (2 * pow(m,3) * xC ) / ( 1+m_2 ) + 2*yC);
        x2 = 0.5 * ( sqrt( d_2 * ( 1 + m_2 )) / ( 1 + m_2 ) + 4*xC -
                2 * xC / (1 + m_2) - 2 * m_2 * xC / (1 + m_2));
        y2 = m * sqrt(d_2 * (1 + m_2)) * 0.5 / (1 + m_2) +
                m * xC - m * xC / (1 + m_2) - pow(m,3) * xC / (1 + m_2)
                + yC;
    }
    if (orizontal)
    {
        xC = this->centroid().x();
        yC = this->centroid().y();
        x1 = xC + 0.5 * DOOR_P_SEGMENT;
        y1 = yC;
        x2 = xC - 0.5 * DOOR_P_SEGMENT;
        y2 = yC;
    }
    if (vertical)
    {
        xC = this->centroid().x();
        yC = this->centroid().y();
        x1 = xC;
        y1 = yC + 0.5 * DOOR_P_SEGMENT;
        x2 = xC;
        y2 = yC - 0.5 * DOOR_P_SEGMENT;
    }
    return SLAM::Geometry::LineSegment(x1,y1,x2,y2);
}

void DoorLineSegment::setFirstRoom(Room *froom)
{
    first = froom;
}

void DoorLineSegment::setSecondRoom(Room *sroom)
{
    second = sroom;
}

bool DoorLineSegment::isFirstOK()
{
    return (second != NULL);
}
bool DoorLineSegment::isSecondOK()
{
    return (first != NULL);
}
bool DoorLineSegment::isAllSet()
{
    return ((first != NULL)&&(second != NULL));
}




}
}
