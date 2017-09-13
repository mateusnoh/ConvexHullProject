#include <Windows.h>
#include <iostream>
#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <vector>
#include <math.h>
#include <chrono>
#include <glut.h>
#include <glm/glm.hpp>

#define PI 3.14159265
#define numPoints 50

using namespace std;
using namespace chrono;

//Convex Hull Project
//by Mateus Ribeiro da Silva
//Unisinos 2016/01

vector<glm::vec2> points;
vector<int> triangles_ids;
vector<glm::vec2> triangles_points;

vector<glm::vec2> lines_hull;

int selected = 0;

bool tempTest = false;

void quickSort(vector<glm::vec2> *data, int first, int last)
{
	glm::vec2 pivot = data->at((first + last) / 2);
	int i = first;
	int j = last;

	while (i <= j)
	{
		while (data->at(i).y < pivot.y && i < last)
		{
			i++;
		}
		while (data->at(j).y > pivot.y && j > first)
		{
			j--;
		}
		if (i <= j)
		{
			glm::vec2 temp = data->at(i);
			data->at(i) = data->at(j);
			data->at(j) = temp;
			i++;
			j--;
		}
	}
	if (j > first)
	{
		quickSort(data, first, j);
	}
	if (i < last)
	{
		quickSort(data, i, last);
	}
}

int orientation(glm::vec2 p, glm::vec2 q, glm::vec2 r)
{
	int val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

	if (val == 0) return 0;  // colinear
	if (val > 0) return 1;	//clock
	if (val < 0) return 2;	//counterclock wise
}

int sqrDist(glm::vec2 a, glm::vec2 b)
{
	int dx = a.x - b.x, dy = a.y - b.y;
	return dx * dx + dy * dy;
}

float sign(glm::vec2 p1, glm::vec2 p2, glm::vec2 p3)
{
	return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
}

int compare(const void *vp1, const void *vp2)
{
	glm::vec2 *p1 = (glm::vec2 *)vp1;
	glm::vec2 *p2 = (glm::vec2 *)vp2;

	// Find orientation
	int o = orientation(points[0], *p1, *p2);
	if (o == 0) return (sqrDist(points[0], *p2) >= sqrDist(points[0], *p1)) ? -1 : 1;

	return (o == 2) ? -1 : 1;
}

float angleBetweenLines(glm::vec2 p1, glm::vec2 p2, glm::vec2 p3, glm::vec2 p4)
{
	float dx1 = p2.x - p1.x;
	float dy1 = p2.y - p1.y;
	float dx2 = p4.x - p3.x;
	float dy2 = p4.y - p3.y;

	float d = dx1*dx2 + dy1*dy2;   // dot product of the 2 vectors
	float l2 = (dx1*dx1+dy1*dy1)*(dx2*dx2+dy2*dy2); // product of the squared lengths

	float param = acos(d/sqrt(l2));
	float angle = atan(param) * 180 / PI;

	return angle;
}

bool PointInTriangle(glm::vec2 * tp1, glm::vec2 * tp2, glm::vec2 * tp3, glm::vec2 * p)
{
	bool b1, b2, b3;

	b1 = sign(*p, *tp1, *tp2) < 0.0f;
	b2 = sign(*p, *tp2, *tp3) < 0.0f;
	b3 = sign(*p, *tp3, *tp1) < 0.0f;
	if (*tp1 == *p || *tp2 == *p || *tp3 == *p)
	{
		return false;
	}
	return ((b1 == b2) && (b2 == b3));
}

bool LineCollision(glm::vec2 p1, glm::vec2 p2, glm::vec2 p3, glm::vec2 p4)
{
	if (p1 == p3 || p1 == p4)
	{
		if (p2 != p3 && p2 != p4)
			return false;
	}
	else if (p2 == p3 || p2 == p4)
	{
		if (p1 != p3 && p1 != p4)
			return false;
	}

	float denominator = ((p2.x - p1.x) * (p4.y - p3.y)) - ((p2.y - p1.y) * (p4.x - p3.x));
	float numerator1 = ((p1.y - p3.y) * (p4.x - p3.x)) - ((p1.x - p3.x) * (p4.y - p3.y));
	float numerator2 = ((p1.y - p3.y) * (p2.x - p1.x)) - ((p1.x - p3.x) * (p2.y - p1.y));

	// Detect coincident lines (has a problem, read below)
	//if (denominator == 0) return numerator1 == 0 && numerator2 == 0;

	float r = numerator1 / denominator;
	float s = numerator2 / denominator;

	return (r >= 0 && r <= 1) && (s >= 0 && s <= 1);
}

bool TriangleCollision(glm::vec2 * t1p1, glm::vec2 * t1p2, glm::vec2 * t1p3, glm::vec2 * t2p1, glm::vec2 * t2p2, glm::vec2 * t2p3)
{
	bool ln1 = false;
	bool ln2 = false;
	bool ln3 = false;

	float angles[9];

	//test 1 - 1/2
	if (LineCollision(*t1p1, *t1p2, *t2p1, *t2p2))
	{
		ln1 = true;
	}
	else if (LineCollision(*t1p1, *t1p2, *t2p1, *t2p3))
	{
		ln1 = true;
	}
	else if (LineCollision(*t1p1, *t1p2, *t2p2, *t2p3))
	{
		ln1 = true;
	}

	//test 2 - 1/3
	if (LineCollision(*t1p1, *t1p3, *t2p1, *t2p3))
	{
		ln2 = true;
	}
	else if (LineCollision(*t1p1, *t1p3, *t2p2, *t2p3))
	{
		ln2 = true;
	}
	else if (LineCollision(*t1p1, *t1p3, *t2p1, *t2p2))
	{
		ln2 = true;
	}

	//test 3 - 2/3
	if (LineCollision(*t1p2, *t1p3, *t2p2, *t2p3))
	{
		ln3 = true;
	}
	else if (LineCollision(*t1p2, *t1p3, *t2p1, *t2p3))
	{
		ln3 = true;
	}
	else if (LineCollision(*t1p2, *t1p3, *t2p1, *t2p2))
	{
		ln3 = true;
	}

	int angleCount = 0;

	if (*t1p1 == *t2p1)
		angleCount++;
	if (*t1p1 == *t2p2)
		angleCount++;
	if (*t1p1 == *t2p3)
		angleCount++;

	if (*t1p2 == *t2p1)
		angleCount++;
	if (*t1p2 == *t2p2)
		angleCount++;
	if (*t1p2 == *t2p3)
		angleCount++;

	if (*t1p3 == *t2p1)
		angleCount++;
	if (*t1p3 == *t2p2)
		angleCount++;
	if (*t1p3 == *t2p3)
		angleCount++;

	if (angleCount >= 3)
	{
		return true;
	}

	if (!ln1 && !ln2 && !ln3)
	{
		return false;
	}

	return true;
}

vector<glm::vec2> convexHullGraham(vector<glm::vec2> *vp)
{
	cout << "Convex Hull Graham Scan..." << endl;
	quickSort(vp, 0, vp->size()-1);

	//Sort by polar angle starting from pivot
	qsort(&points[1], points.size() - 1, sizeof(glm::vec2), compare);

	vector<glm::vec2> hull;
	hull.push_back(points[0]);
	hull.push_back(points[1]);
	hull.push_back(points[2]);

	for (unsigned int i = 3; i < points.size(); i++) 
	{
		glm::vec2 back = hull.back();
		hull.pop_back();
		while (orientation(hull.back(), back, points[i]) != 2) 
		{
			back = hull.back();
			hull.pop_back();
		}
		hull.push_back(back);
		hull.push_back(points[i]);
	}

	//Transfer points to line
	std::vector<glm::vec2> lines_hull_temp;
	for (unsigned int i = 0; i < hull.size(); i++)
	{
		if (i < hull.size() - 1)
		{
			lines_hull_temp.push_back(hull.at(i));
			lines_hull_temp.push_back(hull.at(i + 1));
		}
		else
		{
			lines_hull_temp.push_back(hull.at(i));
			lines_hull_temp.push_back(hull.at(0));
		}
	}
	hull.clear();
	return lines_hull_temp;
}

vector<glm::vec2> convexHullJarvis(vector<glm::vec2> *vp)
{
	vector<glm::vec2> hullTemp;
	cout << "Convex Hull Jarvis..." << endl;
	//Find the leftmost point
	int pivot = 0;
	for (int i = 1; i < vp->size(); i++)
	{
		if (vp->at(i).x < vp->at(pivot).x)
		{
			pivot = i;
		}
	}
	//Start from leftmost point, keep moving counterclockwise
	int posPointHull = pivot;
	int posPointTest;
	do
	{
		// Add current point to result
		hullTemp.push_back(vp->at(posPointHull));

		// Search for a point 'q' such that orientation(p, x, q) is counterclockwise for all points 'x'.
		posPointTest = (posPointHull + 1) % vp->size();
		for (int i = 0; i < vp->size(); i++)
		{
			if (orientation(vp->at(posPointHull), vp->at(i), vp->at(posPointTest)) == 2)
			{
				posPointTest = i;
			}
		}
		posPointHull = posPointTest;

	} while (posPointHull != pivot);  // While we don't come to first point

	//Transfer points to line
	vector<glm::vec2> hull;
	for (unsigned int i = 0; i < hullTemp.size(); i++)
	{
		if (i < hullTemp.size() - 1)
		{
			glm::vec2 temp;
			temp.x = hullTemp.at(i).x;
			temp.y = hullTemp.at(i).y;
			hull.push_back(temp);
			temp.x = hullTemp.at(i + 1).x;
			temp.y = hullTemp.at(i + 1).y;
			hull.push_back(temp);
		}
		else
		{
			glm::vec2 temp;
			temp.x = hullTemp.at(i).x;
			temp.y = hullTemp.at(i).y;
			hull.push_back(temp);
			temp.x = hullTemp.at(0).x;
			temp.y = hullTemp.at(0).y;
			hull.push_back(temp);
		}
	}
	hullTemp.clear();
	return hull;
}

vector<glm::vec2> convexHullBruteforce(vector<glm::vec2> *vp)
{
	cout << "Convex Hull Bruteforce..." << endl;
	//Connect points
	vector<glm::vec2> linesTemp;
	vector<glm::vec2> lines_hull_temp;
	for (unsigned int i = 0; i < vp->size(); i++)
	{
		for (unsigned int j = 0; j < vp->size(); j++)
		{
			if (j != i)
			{
				linesTemp.push_back(vp->at(i));
				linesTemp.push_back(vp->at(j));
			}
		}
	}

	//Compare each line with every point
	bool edge = true;
	bool left = false;
	bool right = false;

	for (unsigned int i = 0; i < linesTemp.size() - 1; i+=2)
	{
		for (unsigned int j = 0; j < vp->size(); j++)
		{
			//Search for orientation
			int result = orientation(linesTemp.at(i), linesTemp.at(i + 1), vp->at(j));
			if (result == 1)
			{
				left = true;
			}
			
			if (result == 2)
			{
				right = true;
			}

			//Dont add line
			if (left == true && right == true)
			{
				edge = false;
			}
		}

		if (edge == true)
		{
			lines_hull_temp.push_back(linesTemp.at(i));
			lines_hull_temp.push_back(linesTemp.at(i + 1));
			edge = true;
			left = false;
			right = false;
		}
		else if (edge == false)
		{
			edge = true;
			left = false;
			right = false;
		}
	}

	return lines_hull_temp;
}

vector<glm::vec2> triangulationBruteforce2D(vector<glm::vec2>* vp)
{
	struct triangle
	{
		glm::vec2 point1, point2, point3;
	};
	vector<glm::vec2> lines;
	vector<triangle> triangles;

	lines = convexHullJarvis(vp);
	//Copy points inside the convex hull
	vector<glm::vec2> notHull;
	vector<glm::vec2> linesTemp = lines;
	for (int i = 0; i < vp->size(); i++)
	{
		bool checkNotHull = true;
		for (int j = 0; j < lines.size() - 1; j += 2)
		{
			if (vp->at(i) == lines.at(j) || vp->at(i) == lines.at(j + 1))
				checkNotHull = false;
		}
		if (checkNotHull)
		{
			notHull.push_back(vp->at(i));
		}
	}

	//Start Triangles
	cout << "Triangulation Bruteforce..." << endl;
	for (int i = 0; i < linesTemp.size() - 1; i += 2)
	{
		triangle triangleTemp;
		int sqrDistLine = 0;
		bool triangleNew = false;
		//Create all triangles as possible from lines
		for (int j = 0; j < notHull.size(); j++)
		{
			if (linesTemp.at(i) != notHull.at(j) && linesTemp.at(i + 1) != notHull.at(j))
			{
				triangle tri;
				tri.point1 = linesTemp.at(i);
				tri.point2 = linesTemp.at(i + 1);
				tri.point3 = notHull.at(j);
				glm::vec2 pm;
				pm.x = (tri.point1.x + tri.point2.x) / 2;
				pm.y = (tri.point1.y + tri.point2.y) / 2;

				int sqrDistLineTemp = sqrDist(pm, tri.point3);

				//Check if triangle is avaliable
				bool triangleCheck = false;
				//Check if no points inside
				if (!triangleCheck)
				{
					for (int k = 0; k < notHull.size(); k++)
					{
						if (PointInTriangle(&tri.point1, &tri.point2, &tri.point3, &notHull.at(k)))
						{
							triangleCheck = true;
						}
					}
				}
				//Check if doesn't collide with others
				if (!triangleCheck)
				{
					for (int k = 0; k < triangles.size(); k++)
					{
						if (TriangleCollision(&tri.point1, &tri.point2, &tri.point3, &triangles.at(k).point1, &triangles.at(k).point2, &triangles.at(k).point3))
						{
							triangleCheck = true;
						}
					}
				}

				if (!triangleCheck)
				{
					if (sqrDistLine == 0)
					{
						sqrDistLine = sqrDistLineTemp;
						triangleTemp.point1 = tri.point1;
						triangleTemp.point2 = tri.point2;
						triangleTemp.point3 = tri.point3;
						triangleNew = true;
					}
					else if (sqrDistLineTemp <= sqrDistLine)
					{
						sqrDistLine = sqrDistLineTemp;
						triangleTemp.point1 = tri.point1;
						triangleTemp.point2 = tri.point2;
						triangleTemp.point3 = tri.point3;
						triangleNew = true;
					}
				}
			}
		}

		if (triangleNew)
		{
			//Add new lines to search for nmore triangle
			glm::vec2 temp;
			temp = triangleTemp.point1;
			linesTemp.push_back(temp);
			temp = triangleTemp.point3;
			linesTemp.push_back(temp);

			temp = triangleTemp.point2;
			linesTemp.push_back(temp);
			temp = triangleTemp.point3;
			linesTemp.push_back(temp);

			triangles.push_back(triangleTemp);
		}
	}
	cout << "Number of Triangles: " << triangles.size() << endl;

	vector<glm::vec2> temp;
	for (int i = 0; i < triangles.size(); i++)
	{
		temp.push_back(triangles.at(i).point1);
		temp.push_back(triangles.at(i).point2);
		temp.push_back(triangles.at(i).point3);
	}
	return temp;
}

vector<int> gridTriangulation2D(int width, int height)
{
	vector<int> triangles;

	for (int i = 0; i < height - 1; i++)
	{
		for (int j = 0; j < width - 1; j++)
		{
			triangles.push_back(j + width * i);
			triangles.push_back(j + width + width * i);
			triangles.push_back(j + 1 + width * i);

			triangles.push_back(j + 1 + width * i);
			triangles.push_back(j + 1 + width + width * i);
			triangles.push_back(j + width + width * i);
		}
	}

	return triangles;
}

void displayText(float x, float y, int r, int g, int b, const char *string) 
{
	int j = strlen(string);

	glColor3f(r, g, b);
	glRasterPos2f(x, y);
	for (int i = 0; i < j; i++) 
	{
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, string[i]);
	}
}

void keyboard (unsigned char key, int x, int y) 
{
	switch (key) 
	{
		case 'Q':
		case 'q':
			exit(0);
		case '0':
			lines_hull.clear();
			triangles_points.clear();
			points.clear();
			srand(time(NULL));
			for (int i = 0; i < numPoints; i++)
			{
				glm::vec2 temp;
				temp.x = rand() % 500;
				temp.y = rand() % 500;
				points.push_back(temp);
			}
			break;
		case '1':
			lines_hull.clear();
			lines_hull = convexHullBruteforce(&points);
			break;
		case '2':
			lines_hull.clear();
			lines_hull = convexHullGraham(&points);
			break;
		case '3':
			lines_hull.clear();
			lines_hull = convexHullJarvis(&points);
			break;
		case '4':
			triangles_points.clear();
			triangles_points = triangulationBruteforce2D(&points);
			break;
		case 'N':
		case 'n':
			selected += 3;
			if (selected <= triangles_points.size() - 3)
			{
				cout << "Selected triangle: " << selected << endl;
				cout << "Point 01: " << triangles_points.at(selected).x << " " << triangles_points.at(selected).y << endl;
				cout << "Point 02: " << triangles_points.at(selected + 1).x << " " << triangles_points.at(selected + 1).y << endl;
				cout << "Point 03: " << triangles_points.at(selected + 2).x << " " << triangles_points.at(selected + 2).y << endl;
			}
			break;
		case 'B':
		case 'b':
			selected -= 3;
			if (selected <= triangles_points.size() - 3)
			{
				cout << "Selected triangle: " << selected << endl;
				cout << "Point 01: " << triangles_points.at(selected).x << " " << triangles_points.at(selected).y << endl;
				cout << "Point 02: " << triangles_points.at(selected + 1).x << " " << triangles_points.at(selected + 1).y << endl;
				cout << "Point 03: " << triangles_points.at(selected + 2).x << " " << triangles_points.at(selected + 2).y << endl;
			}
			break;
	}
	glutPostRedisplay();
}

void display(void)
{
	glClear (GL_COLOR_BUFFER_BIT);
	
	glColor3f(1.0, 1.0, 1.0);
    glBegin(GL_POINTS);
	for(unsigned int i = 0; i < points.size(); i++)
	{
		glVertex3f(points.at(i).x, points.at(i).y, 0);
	}
	glEnd();

	glColor3f(0.0, 0.0, 1.0);
	if(triangles_ids.size() != 0)
	for (unsigned int i = 0; i < triangles_ids.size()-2; i+=3)
	{
		glBegin(GL_LINE_LOOP);
		glVertex2i(points.at(triangles_ids.at(i)).x, points.at(triangles_ids.at(i)).y);
		glVertex2i(points.at(triangles_ids.at(i + 1)).x, points.at(triangles_ids.at(i + 1)).y);
		glVertex2i(points.at(triangles_ids.at(i + 2)).x, points.at(triangles_ids.at(i + 2)).y);
		glEnd();
	}

	if(triangles_points.size() != 0)
	if (selected <= triangles_points.size() - 3)
	{
		glColor3ub(0, 0, 255);
		for (int i = 0; i < triangles_points.size()-2; i+=3)
		{
			glBegin(GL_LINE_LOOP);
			glVertex2i(triangles_points.at(i).x, triangles_points.at(i).y);
			glVertex2i(triangles_points.at(i + 1).x, triangles_points.at(i + 1).y);
			glVertex2i(triangles_points.at(i + 2).x, triangles_points.at(i + 2).y);
			glEnd();
		}
		glColor3ub(0, 255, 0);
		glBegin(GL_LINE_LOOP);
		glVertex2i(triangles_points.at(selected).x, triangles_points.at(selected).y);
		glVertex2i(triangles_points.at(selected + 1).x, triangles_points.at(selected + 1).y);
		glVertex2i(triangles_points.at(selected + 2).x, triangles_points.at(selected + 2).y);
		glEnd();
		glColor3ub(255, 0, 0);
		glBegin(GL_POINTS);
		glVertex2i(triangles_points.at(selected).x, triangles_points.at(selected).y);
		glVertex2i(triangles_points.at(selected + 1).x, triangles_points.at(selected + 1).y);
		glVertex2i(triangles_points.at(selected + 2).x, triangles_points.at(selected + 2).y);
		glEnd();
	}

	glColor3ub(0, 255, 255);
	glBegin(GL_LINE_LOOP);
	for (unsigned int i = 0; i < lines_hull.size(); i++)
	{
		glVertex2i(lines_hull.at(i).x, lines_hull.at(i).y);
	}
	glEnd();

	displayText(0, -20, 0, 0, 0, "2-Graham Scan   3-Jarvis   4-Triangulation   0-Reset Everything   Q-Exit");
	displayText(0, -40, 0, 0, 0, "N-Select Next   B-Select Back");

	glFlush();
}

void init (void) 
{
	// select clearing (background) color
	glClearColor(0.5, 0.5, 0.5, 0.0f);

	glPointSize(5.0);
	glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
	gluOrtho2D(-10, 500,-50, 500);

	srand (time(NULL));
	for (int i = 0; i < numPoints; i++)
	{
		glm::vec2 temp;
		temp.x = rand() % 500;
		temp.y = rand() % 500;
		points.push_back(temp);
	}

	std::cout << "Points: " << points.size() << std::endl;
	triangles_points = triangulationBruteforce2D(&points);
	
}

int main(int argc, char** argv)
{
	glutInit(&argc, argv);
    glutInitDisplayMode (GLUT_SINGLE | GLUT_RGB);
    glutInitWindowSize (800, 600); 
    glutInitWindowPosition (400, 100);
    glutCreateWindow ("Exercícios");
    init ();
    glutDisplayFunc(display); 
	glutKeyboardFunc(keyboard);
    glutMainLoop();
    return 0;
}