#include <GLFW/glfw3.h>
#include <vector>
#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <stack>
#include <chrono>
#include <math.h>
#include <cstdlib>
#include <ctime>



# define M_PI           3.14159265358979323846 

template <typename T>
T max(T a, T b) {
    return (a > b) ? a : b;
}

template <typename T>
T min(T a, T b) {
    return (a < b) ? a : b;
}

const double _EPS = 0.001;
bool spinflg = false;
bool restart = false;
bool recflg  = false;

class Point
{
public:
    double x;
    double y;

    bool isAlive;

    Point() { x = 0.0; y = 0.0; }
    Point(double x, double y) : x(x), y(y) { isAlive = true; }

    bool InPolygon(const std::vector<Point>& vertices)
    {
        int n = vertices.size();
        int count = 0;
        double min_distance = 10;
        for (int i = 0; i < n; ++i)
        {
            Point v1 = vertices[i];
            Point v2 = vertices[(i + 1) % n];

            // Checking whether a horizontal ray from point p intersects a polygon edge
            if (((v1.y > y) != (v2.y > y)) 
                && (x < (v2.x - v1.x) * (y - v1.y) / (v2.y - v1.y) + v1.x))
                count++;

            double A = v2.y - v1.y;
            double B = v1.x - v2.x;
            double C = v2.x * v1.y - v1.x * v2.y;

            double distance = std::abs(A * x + B * y + C) / std::sqrt(A * A + B * B);

            min_distance = min(distance, min_distance);
        }
        // A point inside a polygon if the number of intersections is odd

        

        return (count % 2 == 1) || (min_distance < 0.005);
    }

    bool operator == (const Point& point) const
    {
        return (x == point.x) && (y == point.y);
    }
};

void drawLine(double x1, double y1, double x2, double y2);
void trimPoints(std::vector<Point>& points);

int orientation(const Point& p, const Point& q, const Point& r) {
    double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (fabs(val) < 1e-10) return 0;  
    return (val > 0) ? 1 : 2; 
}

bool doIntersect(const Point& p1, const Point& q1, const Point& p2, const Point& q2) {
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    if (o1 != o2 && o3 != o4)
        return true;

    return false;
}

class Line
{
public:
    Point start;
    Point end;

    Line(Point start, Point end) : start(start), end(end) {}
    Line() {}
    void draw()
    {
        drawLine(start.x, start.y, end.x, end.y);
    }

    bool checkPoint(const Point& point, const Line& line) const
    {
        if (line.start.x < line.end.x)
            if (line.start.y < line.end.y)
                if (point.x <= line.end.x && point.x >= line.start.x && point.y <= line.end.y && point.y >= line.start.y)
                    return true;
        if (line.start.x < line.end.x)
            if (line.start.y > line.end.y)
                if (point.x <= line.end.x && point.x >= line.start.x && point.y >= line.end.y && point.y <= line.start.y)
                    return true;
        if (line.start.x > line.end.x)
            if (line.start.y < line.end.y)
                if (point.x >= line.end.x && point.x <= line.start.x && point.y <= line.end.y && point.y >= line.start.y)
                    return true;
        if (line.start.x > line.end.x)
            if (line.start.y > line.end.y)
                if (point.x >= line.end.x && point.x <= line.start.x && point.y >= line.end.y && point.y <= line.start.y)
                    return true;
        return false;
    }

    void checkIntersections(const Line& line, std::vector<Point>& intersections) const
    {
        double x1, x2, x3, x4, y1, y2, y3, y4;

        x1 = start.x, x2 = end.x, y1 = start.y, y2 = end.y, x3 = line.start.x, x4 = line.end.x, y3 = line.start.y, y4 = line.end.y;
        double a1 = y2 - y1;
        double b1 = x1 - x2;
        double c1 = a1 * x1 + b1 * y1;

        double a2 = y4 - y3;
        double b2 = x3 - x4;
        double c2 = a2 * x3 + b2 * y3;

        double delta = a1 * b2 - a2 * b1;

        if (delta == 0) {
            return;
        }

        double x = (b2 * c1 - b1 * c2) / delta;
        double y = (a1 * c2 - a2 * c1) / delta;

        intersections.push_back(Point(x, y));
    } 

    bool checkIntersections(const std::vector<Point>& vertices, std::vector<Point>& intersections) const
    {
        double x1, x2, x3, x4, y1, y2, y3, y4;
        int n = vertices.size();
        for (int i = 0; i < n; i ++)
        { 
            x1 = start.x, x2 = end.x, y1 = start.y, y2 = end.y, x3 = vertices[i].x, x4 = vertices[(i + 1) % n].x, y3 = vertices[i].y, y4 = vertices[(i + 1) % n].y;
            double a1 = y2 - y1;
            double b1 = x1 - x2;
            double c1 = a1 * x1 + b1 * y1;

            double a2 = y4 - y3;
            double b2 = x3 - x4;
            double c2 = a2 * x3 + b2 * y3;

            double delta = a1 * b2 - a2 * b1;

            if (delta == 0) {
                continue;
            }
            if ((x1 == x2 == x3 == x4) || (y1 == y2 == y3 == y4))
                continue;

            double x = (b2 * c1 - b1 * c2) / delta;
            double y = (a1 * c2 - a2 * c1) / delta;
            if(checkPoint(Point(x,y), Line(Point(vertices[i].x, vertices[i].y), Point(vertices[i + 1].x, vertices[i + 1].y))))
                intersections.push_back(Point(x, y));
        } 
        if (intersections.size() > 0)
            return true;
        return false;
    }
};

class Rectangle
{
public:
    Point tl;
    Point tr;
    Point bl;
    Point br;
    Point center;
    bool isAlive;

    Rectangle() {}

    Rectangle(Point tl, Point tr, Point bl, Point br) : tl(tl), tr(tr), bl(bl), br(br)
    {
        center = Point(tl.x + (tr.x - tl.x) / 2, tl.y + (bl.y - tl.y) / 2);
        isAlive = true;
    }

    double area() const {
        return abs((br.x - bl.x) * (bl.y - tl.y));
    }

    bool isDiagonalIntersectingLine(const std::vector<Point> vertices) 
    { 
        int n = vertices.size();

        for (int i = 0; i < n; i++)
        {
            if (doIntersect(Point(tl.x + 0.003, tl.y + 0.003), Point(br.x - 0.003, br.y - 0.003), vertices[i], vertices[(i + 1) % n]) &&
                doIntersect(Point(tr.x - 0.003, tr.y + 0.003), Point(bl.x + 0.003, bl.y - 0.003), vertices[i], vertices[(i + 1) % n]))
                return true;
        }


        return false;
    }
};

bool isAdjacentVertical(const Rectangle& rect1, const Rectangle& rect2) {
    return (rect1.tl == rect2.bl && rect1.tr == rect2.br) && rect1.isAlive && rect2.isAlive;
}

bool isAdjacentHorizontal(const Rectangle& rect1, const Rectangle& rect2) {
    return (rect1.tr == rect2.tl && rect1.br == rect2.bl) && rect1.isAlive && rect2.isAlive;
}

void trimLines(std::vector<Line>& lines)
{
    for (int i = 0; i < lines.size(); i++)
    {
        for (int j = i + 1; j < lines.size(); j++)
        {
            if (lines[i].start == lines[j].start && lines[i].end == lines[j].end)
            {
                lines.erase(lines.begin() + j), j--;
            }
        }
    }
}

void trimLinesO(std::vector<Line>& lines, int sw)
{
    switch (sw)
    {
    case 1:
        for (int i = 0; i < lines.size(); i++)
        {
            for (int j = i + 1; j < lines.size(); j++)
            {
                if (abs(lines[i].start.x - lines[j].start.x) < 0.3)
                {
                    lines.erase(lines.begin() + j), j--;
                }
            }
        }
        break;
    case 2:
        for (int i = 0; i < lines.size(); i++)
        {
            for (int j = i + 1; j < lines.size(); j++)
            {
                if (abs(lines[i].start.y - lines[j].start.y) < 0.3)
                {
                    lines.erase(lines.begin() + j), j--;
                }
            }
        }
        break;
    }
}

void processLinesV(const std::vector<Point>& vertices, const Point& point, std::vector<Line>& linesV, std::vector<Line>& linesH, int count);
void processLinesH(const std::vector<Point>& vertices, const Point& point, std::vector<Line>& linesV, std::vector<Line>& linesH, int count);

void processLines2(const std::vector<Point>& vertices, const Point& point, std::vector<Line>& linesV, std::vector<Line>& linesH)
{
    int count = 1;
    std::vector<Point> intersections_h;
    std::vector<Point> intersections_v;
    Point temp, p1, p2;

    Line line_v = Line(Point(point.x, 0.0), Point(point.x, 1000.0));

    Line line_h = Line(Point(0.0, point.y), Point(1000.0, point.y));

    line_h.checkIntersections(vertices, intersections_h);

    std::sort(intersections_h.begin(), intersections_h.end(), [](const Point& a, const Point& b) {
        return a.x < b.x;
        });

    for (int i = 0; intersections_h.size() > 0 && i < intersections_h.size() - 1; i++)
    {
        p1 = Point(intersections_h[i].x, intersections_h[i].y);
        p2 = Point(intersections_h[i + 1].x, intersections_h[i + 1].y);
        if (Point(intersections_h[i].x + 0.1, intersections_h[i].y).InPolygon(vertices))
        {
            if(p1.y == p2.y)
                linesH.push_back(Line(p1, p2));
        }
    }
    for(int i = 0; i < intersections_h.size(); i ++)
    {
        processLinesV(vertices, intersections_h[i], linesV, linesH, count);
    }

    line_v.checkIntersections(vertices, intersections_v);

    std::sort(intersections_v.begin(), intersections_v.end(), [](const Point& a, const Point& b) {
        return a.y > b.y;
        });

    for (int i = 0; intersections_v.size() > 0 && i < intersections_v.size() - 1; i++)
    {
        p1 = Point(intersections_v[i].x, intersections_v[i].y);
        p2 = Point(intersections_v[i + 1].x, intersections_v[i + 1].y);
        if (Point(intersections_v[i].x, intersections_v[i].y - 0.1).InPolygon(vertices))
        {
            if(p1.x == p2.x)
                linesV.push_back(Line(p1, p2));
        }
    }
    for (int i = 0; intersections_v.size() > 0 && i < intersections_v.size(); i++)
    {
        processLinesH(vertices, intersections_v[i], linesV, linesH, count);
    }

}

void processLinesV(const std::vector<Point>& vertices, const Point& point, std::vector<Line>& linesV, std::vector<Line>& linesH, int count)
{
    if (count > 2)
        return;
    count++;

    Line line_v = Line(Point(point.x, 0.0), Point(point.x, 1000.0));
    std::vector<Point> intersections_v;

    if (!line_v.checkIntersections(vertices, intersections_v))
        return;
    
    std::sort(intersections_v.begin(), intersections_v.end(), [](const Point& a, const Point& b) {
        return a.y > b.y;
        });
    Point p1, p2;
    for (int i = 0; i < intersections_v.size() - 1; i++)
    {
        if (Point(intersections_v[i].x, intersections_v[i].y - 0.1).InPolygon(vertices))
        {
            p1 = Point(intersections_v[i].x, intersections_v[i].y);
            p2 = Point(intersections_v[i + 1].x, intersections_v[i + 1].y);
            if(p1.x == p2.x)
                linesV.push_back(Line(p1, p2));
        }
    }
    for (int i = 0; i < intersections_v.size(); i++)
    {
        processLinesH(vertices, intersections_v[i], linesV, linesH, count);
    }
}

void processLinesH(const std::vector<Point>& vertices, const Point& point, std::vector<Line>& linesV, std::vector<Line>& linesH, int count)
{
    if (count > 2)
        return;
    count++;

    Line line_h = Line(Point(0.0, point.y), Point(1000.0, point.y));
    std::vector<Point> intersections_h;

    if(!(line_h.checkIntersections(vertices, intersections_h)))
        return;

    std::sort(intersections_h.begin(), intersections_h.end(), [](const Point& a, const Point& b) {
        return a.x < b.x;
        });

    Point p1, p2;
    for (int i = 0; i < intersections_h.size() - 1; i++)
    {
        if (Point(intersections_h[i].x + 0.1, intersections_h[i].y).InPolygon(vertices))
        {
            p1 = Point(intersections_h[i].x, intersections_h[i].y);
            p2 = Point(intersections_h[i + 1].x, intersections_h[i + 1].y);
            if(p1.y == p2.y)
                linesH.push_back(Line(p1, p2));
        }
    }
    for (int i = 0; i < intersections_h.size(); i++)
    {
        processLinesV(vertices, intersections_h[i], linesV, linesH, count);
    }
}

void trimPoints(std::vector<Point>& points)
{
    for (int i = 0; i < points.size(); i++)
        for (int j = i + 1 ; j < points.size(); j++)
            if (abs(points[i].x - points[j].x) <= 0.0001 
                && abs(points[i].y - points[j].y) <= 0.0001)
                points.erase(points.begin() + j), j--;
}

bool makeLineR(const Point& point1, const Point& point2, const std::vector <Point>& vertices, Line& line)
{
    Point checkP = Point(point1.x + 0.1, point1.y);
    if (checkP.InPolygon(vertices))
    {
        line = Line(point1, point2);
        return true;
    }
    return false;
}

bool makeLineL(const Point& point1, const Point& point2, const std::vector <Point>& vertices, Line& line)
{
    Point checkP = Point(point1.x, point1.y + 0.1);
    if (checkP.InPolygon(vertices))
    {
        line = Line(point1, point2);
        return true;
    }
    return false;
}

std::vector<std::vector<Rectangle>> initiateRectanglesByLines2(const std::vector<Line>& linesH, const std::vector<Line>& linesV, const std::vector<Point>& vertices, std::vector<std::vector<Point>>& inter)
{
    std::vector<Point> points;
    std::vector<std::vector<Rectangle>> rectangles;
    std::vector<std::vector<Point>> intersections;

    for (int i = 0; i < linesH.size(); i++)
    {
        points.clear();
        for (int j = 0; j < linesV.size(); j++)
        {
            linesH[i].checkIntersections(linesV[j], points);
        }
        if (points.size() > 1)
            std::sort(points.begin(), points.end(), [](const Point& a, const Point& b) {
            return a.x < b.x;
                });
        intersections.push_back(points);
    }

    for (int i = 0; i < intersections.size(); i++)
        if (intersections[i].size() == 0)
            intersections.erase(intersections.begin() + i), i--;

    std::sort(intersections.begin(), intersections.end(), [](const std::vector<Point>& a, const std::vector<Point>& b) {
        return a[0].y > b[0].y;
        });

    if(intersections.size() > 1)
    {
        for (int i = 0; i < intersections.size() - 1; i++)
        {
            if ((intersections[i][0].y - intersections[i + 1][0].y) < 0.001)
                intersections.erase(intersections.begin() + i), i--;
            else
                trimPoints(intersections[i]);
        }
        trimPoints(intersections[intersections.size() - 1]);
    }

    for (int i = 0; i < intersections.size(); i++)
        for (int j = 0; j < intersections[i].size(); j++)
            if (!intersections[i][j].InPolygon(vertices))
                intersections[i][j].isAlive = false;

    Point tl, tr, bl, br;
    Rectangle rect;

    for (int i = 0; intersections.size() > 0 && i < intersections.size() - 1; i++)
    {
        rectangles.push_back(std::vector<Rectangle>());
        for (int j = 0; intersections[i].size() > 0 && j < intersections[i].size() - 1 && j < intersections[i + 1].size() - 1; j++)
        {
            tl = intersections[i + 1][j]; tr = intersections[i + 1][j + 1]; bl = intersections[i][j]; br = intersections[i][j + 1];
            rect = Rectangle(tl, tr, bl, br);
            rectangles[i].push_back(rect);
            if (!(bl.isAlive && br.isAlive && tl.isAlive && tr.isAlive) || rect.isDiagonalIntersectingLine(vertices))
                rectangles[i][j].isAlive = false;
        }
    }

    inter = intersections;
    return rectangles;
}

// Function for handling errors
void errorCallback(int error, const char* description) {
    std::cerr << "Error: " << description << std::endl;
}

std::vector<Point> convertFromNative(std::vector<double> native, int width, int height)
{
    std::vector<Point> points;
    for (int i = 0; i < native.size(); i+=2)   
        points.push_back(Point(native[i], native[i + 1]));

    return points;
}

// Function for handling window size changes
void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

void drawLine(double x1, double y1, double x2, double y2) {
    glBegin(GL_LINES);
    glVertex2f(x1, y1);
    glVertex2f(x2, y2);
    glEnd();
}

void drawPolygon(const std::vector<Point>& vertices)
{
    int n = vertices.size();
    for (int i = 0; i < n; i ++)
        drawLine(vertices[i].x, vertices[i].y, vertices[(i + 1) % n].x, vertices[(i + 1) % n].y);
}

void drawPolygon(const std::vector<Rectangle>& rectangles)
{
    for (int i = 0; i < rectangles.size(); i++)
    {
        if(rectangles[i].isAlive)
        {
            drawLine(rectangles[i].tl.x, rectangles[i].tl.y, rectangles[i].tr.x, rectangles[i].tr.y);
            drawLine(rectangles[i].tr.x, rectangles[i].tr.y, rectangles[i].br.x, rectangles[i].br.y);
            drawLine(rectangles[i].br.x, rectangles[i].br.y, rectangles[i].bl.x, rectangles[i].bl.y);
            drawLine(rectangles[i].bl.x, rectangles[i].bl.y, rectangles[i].tl.x, rectangles[i].tl.y);
        }
    }
}

void drawPolygon(const std::vector<std::vector<Rectangle>>& rectangles)
{
    for (int i = 0; i < rectangles.size(); i++)
    {
        for(int j = 0; j < rectangles[i].size(); j ++)
        {
            if (rectangles[i][j].isAlive)
            {
                drawLine(rectangles[i][j].tl.x, rectangles[i][j].tl.y, rectangles[i][j].tr.x, rectangles[i][j].tr.y);
                drawLine(rectangles[i][j].tr.x, rectangles[i][j].tr.y, rectangles[i][j].br.x, rectangles[i][j].br.y);
                drawLine(rectangles[i][j].br.x, rectangles[i][j].br.y, rectangles[i][j].bl.x, rectangles[i][j].bl.y);
                drawLine(rectangles[i][j].bl.x, rectangles[i][j].bl.y, rectangles[i][j].tl.x, rectangles[i][j].tl.y);
            }
        }
    }
}

void drawPolygon(const Rectangle rectangle)
{
        drawLine(rectangle.tl.x, rectangle.tl.y, rectangle.tr.x, rectangle.tr.y);
        drawLine(rectangle.tr.x, rectangle.tr.y, rectangle.br.x, rectangle.br.y);
        drawLine(rectangle.br.x, rectangle.br.y, rectangle.bl.x, rectangle.bl.y);
        drawLine(rectangle.bl.x, rectangle.bl.y, rectangle.tl.x, rectangle.tl.y);
}

void drawGrid(int width, int height, double spacing) {
    glColor3f(0.8f, 0.8f, 0.8f); // Grid color (light-grey)

    // Vertical lines
    for (double x = -1.0f; x <= 1.0f; x += spacing) {
        drawLine(x, -1.0f, x, 1.0f);
    }

    // Horizontal lines
    for (double y = -1.0f; y <= 1.0f; y += spacing) {
        drawLine(-1.0f, y, 1.0f, y);
    }

    glColor3f(0.8f, 0.8f, 0.8f); // Axis color (red)

    // Drawing axises
    drawLine(-1.0f, 0.0f, 1.0f, 0.0f); // Axis X
    drawLine(0.0f, -1.0f, 0.0f, 1.0f); // Axis Y
}

void createVectorsHV2(const std::vector<std::vector<int>>& UNI, /*const std::vector<std::vector<int>>& R, const std::vector<std::vector<int>>& L,*/ std::vector<int>& H, std::vector<int>& V, const int n, const int m)
{
    H.clear(), V.clear();
    int value = 0;
    for (int i = n; i < UNI.size() && UNI[i][m] > 0; i++)
    {
        value = 0;
        for (int j = m; j < UNI[0].size(); j++)
        {
            if (UNI[i][j] > 0 && UNI[i][j] != 2)
            {
                value++;
            }
            else
            {
                H.push_back(value);
                break;
            }
        }
    }
}

void createMatricesRV2(const std::vector<std::vector<Rectangle>>& rectanglesM, std::vector<std::vector<int>>& UNI, /*std::vector<std::vector<int>>& L, std::vector<std::vector<int>>& R,*/ int ramount_v, int ramount_h)
{

    bool hor, ver;
    for (int i = 0; rectanglesM.size() - 1 > 0 && i < rectanglesM.size() - 1; i++)
    {
        UNI.push_back(std::vector<int>());
        for (int j = 0; rectanglesM[i].size() - 1 > 0 && j < rectanglesM[i].size() - 1; j++)
        {
            UNI[i].push_back(0);
            hor = isAdjacentHorizontal(rectanglesM[i][j], rectanglesM[i][j + 1]);
            ver = isAdjacentVertical(rectanglesM[i][j], rectanglesM[i + 1][j]);
            if (hor)
                UNI[i][j] = 1;
            if (ver)
                UNI[i][j] = 2;
            if (hor && ver)
                UNI[i][j] = 3;
        }
        UNI[i].push_back(0);
    }
    UNI.push_back(std::vector<int>());

    for (int j = 0;  UNI[UNI.size() - 1].size() && j < UNI[UNI.size() - 1].size(); j++)
    {
        UNI[UNI.size() - 1].push_back(0);
        hor = isAdjacentHorizontal(rectanglesM[rectanglesM.size()][j], rectanglesM[rectanglesM.size()][j + 1]);
        if (hor)
            UNI[UNI.size() - 1][j] = 1;
    }

    for (int i = 0; i < ramount_h; i++)
    {
        UNI[UNI.size() - 1].push_back(0);
    }

}

Rectangle findLargestInscribedRectangle2(const std::vector<std::vector<Rectangle>>& rectanglesM, int N, int M)
{

    std::vector<std::vector<int>> R, L, UNI;
    std::vector<int> H, V;

    createMatricesRV2(rectanglesM, UNI, N, M);

    Rectangle maxRect = rectanglesM[0][0];
    double maxArea = 0.0;
    double area = 0.0;
    int minh;
    int _i = 0, _j = 0, _l = 0, _k = 0;;
    auto start5 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < M; ++j) {
            if (UNI[i][j] > 0) {
                createVectorsHV2(UNI, H, V, i, j);
                minh = H[0];
                if (H[0] != 0) {
                    for (int k = 1; k < H.size(); k++) {
                        if (H[k] >= 1) {
                            if (H[k] < minh)
                                minh = H[k];
                            area = (rectanglesM[i][j].bl.y - rectanglesM[i + k][j].tl.y)
                                * (rectanglesM[i][j + minh].br.x - rectanglesM[i][j].bl.x);
                            if (area > maxArea)
                            {
                                maxArea = area;
                                _i = i, _j = j, _k = k, _l = minh;
                            }
                        }
                    }
                }
            }
        }
    }

    return Rectangle(rectanglesM[_i][_j].bl, rectanglesM[_i][_j + _l].br, rectanglesM[_i + _k][_j].tl, rectanglesM[_i + _k][_j + _l].tr);
}

Point rotatePoint(const Point& p, double angle, const Point& origin) {
    double s = std::sin(angle);
    double c = std::cos(angle);

    double x_new = p.x - origin.x;
    double y_new = p.y - origin.y;

    double x_rot = x_new * c - y_new * s;
    double y_rot = x_new * s + y_new * c;

    x_rot += origin.x;
    y_rot += origin.y;

    return Point(x_rot, y_rot);
}

std::vector<Point> rotatePolygonEdgeToXAxis(const std::vector<Point>& polygon, int edgeStartIdx, const Point& origin) {
    int edgeEndIdx = (edgeStartIdx + 1) % polygon.size();

    Point p1 = polygon[edgeStartIdx];
    Point p2 = polygon[edgeEndIdx];

    double angle = std::atan2(p2.y - p1.y, p2.x - p1.x);

    double rotateAngle = -angle;

    std::vector<Point> rotatedPolygon;
    for (const Point& p : polygon) {
        rotatedPolygon.push_back(rotatePoint(p, rotateAngle, origin));
    }

    return rotatedPolygon;
}

std::vector<Point> rotatePolygonEdgeToYAxis(const std::vector<Point>& polygon, int edgeStartIdx, const Point& origin) {
    int edgeEndIdx = (edgeStartIdx + 1) % polygon.size();

    Point p1 = polygon[edgeStartIdx];
    Point p2 = polygon[edgeEndIdx];

    double angle = std::atan2(p2.y - p1.y, p2.x - p1.x);

    double rotateAngle = M_PI / 2 - angle;

    std::vector<Point> rotatedPolygon;
    for (const Point& p : polygon) {
        rotatedPolygon.push_back(rotatePoint(p, rotateAngle, origin));
    }

    return rotatedPolygon;
}


std::vector<Point> rotatePolygon(const std::vector<Point>& polygon, const Point& center, double angle) {
    std::vector<Point> rotatedPolygon;
    double angleRad = angle * M_PI / 180.0; 

    for (const Point& p : polygon) {

        double x_prime = p.x - center.x;
        double y_prime = p.y - center.y;


        double x_double_prime = x_prime * std::cos(angleRad) - y_prime * std::sin(angleRad);
        double y_double_prime = x_prime * std::sin(angleRad) + y_prime * std::cos(angleRad);


        double x_final = x_double_prime + center.x;
        double y_final = y_double_prime + center.y;

        rotatedPolygon.emplace_back(x_final, y_final);
    }

    return rotatedPolygon;
}

std::vector<Point> generateStarPolygon(int numPoints, double centerX, double centerY, double radius) 
{
    std::vector<Point> points;
    double angleStep = M_PI / numPoints; 

    for (int i = 0; i < 2 * numPoints; ++i) {
        double angle = i * angleStep;
        double r = (i % 2 == 0) ? radius : radius / 2; 
        double x = centerX + r * cos(angle);
        double y = centerY + r * sin(angle);
        points.push_back(Point(x, y));
    }

    return points;
}

double randomDouble(double min, double max) {
    return min + (rand() / (RAND_MAX / (max - min)));
}

std::vector<Point> generateConvexPolygon(int numPoints, double centerX, double centerY, double minRadius, double maxRadius) {
    std::vector<Point> points;
    std::vector<double> angles;

    for (int i = 0; i < numPoints; ++i) {
        double angle = randomDouble(0, 2 * M_PI);
        angles.push_back(angle);
    }

    std::sort(angles.begin(), angles.end());

    for (int i = 0; i < numPoints; ++i) {
        double radius = randomDouble(minRadius, maxRadius);
        double x = centerX + radius * cos(angles[i]);
        double y = centerY + radius * sin(angles[i]);
        points.push_back({ x, y });
    }

    return points;
}

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) 
{
    if (action == GLFW_PRESS) {
        switch (key) {
        case GLFW_KEY_1:
            spinflg = true;
            break;
        case GLFW_KEY_2:
            spinflg = false;
            break;
        case GLFW_KEY_3:
            restart = true;
            break;
        case  GLFW_KEY_4:
            recflg = true;
            break;
        case  GLFW_KEY_5:
            recflg = false;
            break;
        }
    }
}

int main() {
a:
    restart = false;
    int width = 1100;
    int height = 1100;

    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    // Installing error handler
    glfwSetErrorCallback(errorCallback);

    // Creating window
    GLFWwindow* window = glfwCreateWindow(width, height, "Polygon Drawing", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }

    // Settin gup OpenGL window context
    glfwMakeContextCurrent(window);

    // Setting up function for changing window size
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    // Setting up polygon vertices
    /*std::vector<double> vertices = {
         0.0f,  100.0f,
         100.0f,  300.0f,
         400.0f,  400.0f,
         300.0f,  600.0f,
         600.0f,  500.0f,
         800.0f,  700.0f,
         800.0f,  500.0f,
         1000.0f,  500.0f,
         900.0f,  300.0f,
         1000.0f,  0.0f,
         600.0f,  100.0f
    };*/
    
    std::cout << "Choose enter mode: 1 for manual input, 2 for preset polygons or 3 for randomized polygon: " << std::endl;
    int mode;
    std::cin >> mode;
    std::vector<double> vertices;
    std::vector<Point> vertices_p;
    int amount;
    int preset;
    bool circular = false;
    switch (mode)
    {
    case 1:
        std::cout << "Enter points amount: " << std::endl;
        amount = 0;
        std::cin >> amount;
        for (int i = 0; i < amount; i++)
        {
            std::cout << "Enter point " << i << " x: " << std::endl;
            double px;
            std::cin >> px;
            std::cout << "Enter point " << i << " y: " << std::endl;
            double py;
            std::cin >> py;

            vertices.push_back(px);
            vertices.push_back(py);
        }
        vertices.push_back(vertices[0]);
        vertices.push_back(vertices[1]);
        break;
    case 2:
        std::cout << "Choose preset, 1 to 5: " << std::endl;
        std::cin >> preset;
        switch (preset)
        {
        case 1:
            vertices = {
                500.0f, 100.0f,
                600.0f, 250.0f,
                700.0f, 150.0f,
                800.0f, 300.0f,
                850.0f, 500.0f,
                700.0f, 400.0f,
                750.0f, 600.0f,
                600.0f, 500.0f,
                500.0f, 700.0f,
                400.0f, 500.0f,
                250.0f, 600.0f,
                300.0f, 400.0f,
                150.0f, 500.0f,
                200.0f, 300.0f,
                300.0f, 150.0f,
                400.0f, 250.0f,
                500.0f, 100.0f
            };
            break;
        case 2:
            vertices = {
                400.0, 300.0,
                675.0, 450.0,
                370.0, 500.0,
                575.0, 525.0,
                320.0, 725.0,
                590.0, 625.0,
                650.0, 900.0,
                625.0, 650.0,
                800.0, 800.0,
                700.0, 600.0,
                900.0, 575.0,
                750.0, 550.0,
                930.0, 450.0,
                775.0, 475.0,
                400.0, 300.0
            };
            break;
        case 3:
            vertices = {
                480.0f,  100.0f,
                560.0f,  180.0f,
                640.0f,  150.0f,
                720.0f,  240.0f,
                850.0f,  280.0f,
                890.0f,  370.0f,
                820.0f,  430.0f,
                760.0f,  500.0f,
                810.0f,  600.0f,
                750.0f,  680.0f,
                620.0f,  720.0f,
                540.0f,  750.0f,
                470.0f,  710.0f,
                350.0f,  680.0f,
                260.0f,  630.0f,
                320.0f,  540.0f,
                280.0f,  460.0f,
                170.0f,  400.0f,
                240.0f,  340.0f,
                290.0f,  270.0f,
                370.0f,  220.0f,
                450.0f,  200.0f,
                480.0f,  100.0f,
            };
            break;
        case 4:
            vertices = {
                150.0,  310.0,
                210.0, 370.0,
                250.0, 450.0,
                290.0, 540.0,
                340.0, 620.0,
                380.0, 680.0,
                470.0, 725.0,
                540.0, 765.0,
                620.0, 740.0,
                685.0, 670.0,
                760.0, 615.0,
                825.0, 545.0,
                765.0, 465.0,
                830.0, 410.0,
                875.0, 315.0,
                815.0, 275.0,
                735.0, 225.0,
                645.0, 130.0,
                580.0, 190.0,
                495.0, 105.0,
                420.0, 180.0,
                350.0, 250.0,
                270.0, 220.0,
                230.0, 270.0,
                150.0, 310.0
            };
            break;
        case 5:
            vertices = {
                480.0, 110.0,
                540.0, 170.0,
                610.0, 130.0,
                700.0, 210.0,
                780.0, 160.0,
                840.0, 290.0,
                870.0, 350.0,
                810.0, 400.0,
                750.0, 470.0,
                790.0, 570.0,
                720.0, 630.0,
                670.0, 710.0,
                590.0, 740.0,
                530.0, 700.0,
                400.0, 670.0,
                360.0, 610.0,
                310.0, 530.0,
                260.0, 450.0,
                210.0, 380.0,
                170.0, 320.0,
                240.0, 270.0,
                290.0, 200.0,
                360.0, 230.0,
                440.0, 180.0,
                480.0, 110.0
            };
            break;
        }
        break;
        case 3:
            std::cout << "Enter amount of vertices (due to high algorithm complexity, recommended amount of vertices is below 25): " << std::endl;
            int vcount;
            std::cin >> vcount;
            vertices_p = generateStarPolygon(vcount, 500.0, 500.0, 300.0);
            vertices_p.push_back(vertices_p[0]);
            break;
    }


    std::cout << "Do you want to calculate for all edges? Y/n ";
    char circ;
    std::cin >> circ;
    if (circ == 'Y')
        circular = true;

    std::cout << "Calculating in process...." << std::endl;
    if(vertices_p.size() == 0)
        vertices_p = convertFromNative(vertices, width, height);

    

    std::vector<Point> points;
    std::vector<Line> linesVT;
    std::vector<Line> linesHT;

    auto start2 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < vertices_p.size(); i++)
        processLines2(vertices_p, vertices_p[i], linesVT, linesHT);
    trimLines(linesHT); trimLines(linesVT);


    std::vector<std::vector<Rectangle>> rectangles2;
    std::vector<std::vector<Rectangle>> rectangles;

    std::vector<std::vector<Point>> inter;

    rectangles2 = initiateRectanglesByLines2(linesHT, linesVT, vertices_p, inter);

    Rectangle ler2 = findLargestInscribedRectangle2(rectangles2, rectangles2.size(), rectangles2[0].size());

    std::vector<Point> maxVertices = vertices_p;
    Rectangle maxRectangle = ler2;
    std::vector<Rectangle> drectangles;
    std::vector<std::vector<Point>> dvertices;
    std::vector<std::vector<Rectangle>> maxRectangles;
    std::vector<std::vector<std::vector<Rectangle>>> dRectangles;
    
    std::vector<Point> vertices_c = vertices_p;

    if(circular)
    {
        for (int i = 0; i < vertices_p.size(); i++)
        {
            linesVT.clear(); linesHT.clear();
            vertices_c = rotatePolygonEdgeToXAxis(vertices_p, i, Point(500.0, 500.0));

            for (int j = 0; j < vertices_c.size(); j++)
                processLines2(vertices_c, vertices_c[j], linesVT, linesHT);
            trimLines(linesHT);
            trimLines(linesVT);


            std::vector<std::vector<Rectangle>> rectangles2;

            std::vector<std::vector<Point>> inter;

            rectangles2 = initiateRectanglesByLines2(linesHT, linesVT, vertices_c, inter);

            Rectangle leri = findLargestInscribedRectangle2(rectangles2, rectangles2.size(), rectangles2[0].size());
            drectangles.push_back(leri);
            dvertices.push_back(vertices_c);
            dRectangles.push_back(rectangles2);
            if (leri.area() >= maxRectangle.area())
            {
                maxRectangle = leri;
                maxVertices = vertices_c;
                maxRectangles = rectangles2;
            }
        }
        for (int i = 0; i < vertices_p.size(); i++)
        {
            linesVT.clear(); linesHT.clear();
            vertices_c = rotatePolygonEdgeToYAxis(vertices_p, i, Point(500.0, 500.0));

            for (int j = 0; j < vertices_c.size(); j++)
                processLines2(vertices_c, vertices_c[j], linesVT, linesHT);
            trimLines(linesHT);
            trimLines(linesVT);


            std::vector<std::vector<Rectangle>> rectangles2;

            std::vector<std::vector<Point>> inter;

            rectangles2 = initiateRectanglesByLines2(linesHT, linesVT, vertices_c, inter);

            Rectangle leri = findLargestInscribedRectangle2(rectangles2, rectangles2.size(), rectangles2[0].size());
            drectangles.push_back(leri);
            dvertices.push_back(vertices_c);
            dRectangles.push_back(rectangles2);
            if (leri.area() >= maxRectangle.area())
            {
                maxRectangle = leri;
                maxVertices = vertices_c;
                maxRectangles = rectangles2;
            }
        }
    }

    std::cout << "\n...Done" << std::endl;

    ler2 = maxRectangle;
    vertices_p = maxVertices;
    rectangles2 = maxRectangles;
    std::cout << "Max Rectangle area = " << maxRectangle.area() << std::endl;
    int frame_counter = 0;
    int cc = 0;

    glfwSetKeyCallback(window, keyCallback);

    while (!glfwWindowShouldClose(window)) {
        glClear(GL_COLOR_BUFFER_BIT);

        if (spinflg)
        {
            ler2 = drectangles[cc];
            vertices_p = dvertices[cc];
            if (frame_counter > 200)
            {
                cc++;
                if (cc > drectangles.size() - 1)
                    cc = 0;
                ler2 = drectangles[cc];
                vertices_p = dvertices[cc];
                rectangles2 = dRectangles[cc];
                frame_counter = 0;
            }

            frame_counter++;
        }
        else
        {
            ler2 = maxRectangle;
            vertices_p = maxVertices;
            rectangles2 = maxRectangles;
        }

        if (restart)
        {
            glfwDestroyWindow(window);
            glfwTerminate();
            goto a;
        }

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0.0, width, 0.0, height, -1.0, 1.0);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
       
        glColor3f(1.0f, 0.0f, 0.0f); // color red
        drawPolygon(vertices_p);

        if(recflg)
        {
            glColor3f(.0f, 1.0f, 0.0f); // color green
            drawPolygon(rectangles2);
        }


        glColor3f(1.0f, 1.0f, 1.0f); // color white
        glPointSize(5.0f);
        glColor3f(1.0f, 1.0f, 0.0f);
        drawPolygon(ler2);


        // Swapping buffers
        glfwSwapBuffers(window);

        // Events processing
        glfwPollEvents();
    }

    // Cleaning resourses
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
