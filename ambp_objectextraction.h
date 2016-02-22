#ifndef AMBP_OBJECTEXTRACTION_H
#define AMBP_OBJECTEXTRACTION_H

#include "opencv/cv.h"
#include "opencv\cxcore.h"

/*********************************************************************************/
/* AMBP Object */
/*********************************************************************************/

#define DEGREE2RAD		(CV_PI / 180.0)

typedef std::vector<double> double_stl_vector;

#define B_INCLUDE				1L
#define B_EXCLUDE				2L
#define B_EQUAL					3L
#define B_NOT_EQUAL				4L
#define B_GREATER				5L
#define B_LESS					6L
#define B_GREATER_OR_EQUAL		7L
#define B_LESS_OR_EQUAL			8L
#define B_INSIDE			    9L
#define B_OUTSIDE			    10L

#define EXCEPTION_BLOB_OUT_OF_BOUNDS	1000
#define EXCEPCIO_CALCUL_BLOBS			1001

#define B_CONNECTIVITAT_8
#define IMATGE_CICLICA_VERTICAL		1
#define IMATGE_CICLICA_HORITZONTAL	0
#define PERIMETRE_DIAGONAL (1.41421356237310 - 2)
#define SQRT2	1.41421356237310
#define PIXEL_EXTERIOR 0

namespace AMBP_OBJECT
{
    class CBlob
    {
    public:
      CBlob();
      CBlob( const CBlob &src );
      CBlob( const CBlob *src );
      ~CBlob();

      CBlob& operator=(const CBlob &src );
      bool IsEmpty() const
      {
        return (area == 0.0 && perimeter == 0.0 );
      };

      // Clears the edges of the blob
      void ClearEdges();

      // Adds the blob edges to another blob
      void CopyEdges( CBlob &destination ) const;

      // Calculates the convex hull of the blob
      bool GetConvexHull( CvSeq **dst ) const;

      // Fits an ellipse to the blob edges
      CvBox2D GetEllipse() const;

      // Paints the blob in an image
      void FillBlob( IplImage *imatge, CvScalar color, int offsetX = 0, int offsetY = 0 ) const;

      inline int Label() const	{ return etiqueta; }
      inline int Parent() const	{ return parent; }
      inline double Area() const { return area; }
      inline double Perimeter() const { return perimeter; }
      inline double ExternPerimeter() const { return externPerimeter; }
      inline int	  Exterior() const { return exterior; }
      inline double Mean() const { return mean; }
      inline double StdDev() const { return stddev; }
      inline double MinX() const { return minx; }
      inline double MinY() const { return miny; }
      inline double MaxX() const { return maxx; }
      inline double MaxY() const { return maxy; }
      inline CvSeq *Edges() const { return edges; }
      inline double SumX() const { return sumx; }
      inline double SumY() const { return sumy; }
      inline double SumXX() const { return sumxx; }
      inline double SumYY() const { return sumyy; }
      inline double SumXY() const { return sumxy; }

      // label of the blob
      int etiqueta;
      // true for extern blobs
      int exterior;
      // Blob area
      double area;
      // Blob perimeter
      double perimeter;
      // amount of blob perimeter which is exterior
      double externPerimeter;
      // label of the parent blob
      int parent;
      double sumx, sumy, sumxx, sumyy, sumxy, minx, maxx, miny, maxy;

      // mean of the grey scale values of the blob pixels
      double mean;
      // standard deviation of the grey scale values of the blob pixels
      double stddev;

      // storage which contains the edges of the blob
      CvMemStorage *m_storage;
      // Sequence with the edges of the blob
      CvSeq *edges;

      // Point datatype for plotting (FillBlob)
      typedef std::vector<CvPoint> vectorPunts;

      //! Helper class to compare two CvPoints (for sorting in FillBlob)
      struct comparaCvPoint : public std::binary_function<CvPoint, CvPoint, bool>
      {
        bool operator()(CvPoint a, CvPoint b)
        {
          if( a.y == b.y )
            return a.x < b.x;
          else
            return a.y < b.y;
        }
      };
    };



    /**************************************************************************
    Definici?de les classes per a fer operacions sobre els blobs

    Helper classes to perform operations on blobs
    **************************************************************************/


    //! Classe d'on derivarem totes les operacions sobre els blobs
    //! Interface to derive all blob operations
    class COperadorBlob
    {
    public:
      virtual ~COperadorBlob(){};
      virtual double operator()(const CBlob &blob) const = 0;
      virtual const char *GetNom() const = 0;

      operator COperadorBlob*() const
      {
        return (COperadorBlob*)this;
      }
    };

    typedef COperadorBlob funcio_calculBlob;

    #ifdef BLOB_OBJECT_FACTORY

    struct functorComparacioIdOperador
    {
      bool operator()(const char* s1, const char* s2) const
      {
        return strcmp(s1, s2) < 0;
      }
    };

    // Definition of Object factory type for COperadorBlob objects
    typedef ObjectFactory<COperadorBlob, const char *, functorComparacioIdOperador > t_OperadorBlobFactory;

    void RegistraTotsOperadors( t_OperadorBlobFactory &fabricaOperadorsBlob );

    #endif

    //! Class to get the area of a blob
    class CBlobGetArea : public COperadorBlob
    {
    public:
      double operator()(const CBlob &blob) const
      {
        return blob.Area();
      }
      const char *GetNom() const
      {
        return "CBlobGetArea";
      }
    };

    // Class to get the perimeter of a blob
    class CBlobGetPerimeter: public COperadorBlob
    {
    public:
      double operator()(const CBlob &blob) const
      {
        return blob.Perimeter();
      }
      const char *GetNom() const
      {
        return "CBlobGetPerimeter";
      }
    };

    // Class to get the extern flag of a blob
    class CBlobGetExterior: public COperadorBlob
    {
    public:
      double operator()(const CBlob &blob) const
      {
        return blob.Exterior();
      }
      const char *GetNom() const
      {
        return "CBlobGetExterior";
      }
    };

    // Class to get the mean grey level of a blob
    class CBlobGetMean: public COperadorBlob
    {
    public:
      double operator()(const CBlob &blob) const
      {
        return blob.Mean();
      }
      const char *GetNom() const
      {
        return "CBlobGetMean";
      }
    };

    // Class to get the standard deviation of the grey level values of a blob
    class CBlobGetStdDev: public COperadorBlob
    {
    public:
      double operator()(const CBlob &blob) const
      {
        return blob.StdDev();
      }
      const char *GetNom() const
      {
        return "CBlobGetStdDev";
      }
    };

    // Class to calculate the compactness of a blob
    class CBlobGetCompactness: public COperadorBlob
    {
    public:
      double operator()(const CBlob &blob) const;
      const char *GetNom() const
      {
        return "CBlobGetCompactness";
      }
    };

    // Class to calculate the length of a blob
    class CBlobGetLength: public COperadorBlob
    {
    public:
      double operator()(const CBlob &blob) const;
      const char *GetNom() const
      {
        return "CBlobGetLength";
      }
    };

    // Class to calculate the breadth of a blob
    class CBlobGetBreadth: public COperadorBlob
    {
    public:
      double operator()(const CBlob &blob) const;
      const char *GetNom() const
      {
        return "CBlobGetBreadth";
      }
    };

    class CBlobGetDiffX: public COperadorBlob
    {
    public:
      double operator()(const CBlob &blob) const
      {
        return blob.maxx - blob.minx;
      }
      const char *GetNom() const
      {
        return "CBlobGetDiffX";
      }
    };

    class CBlobGetDiffY: public COperadorBlob
    {
    public:
      double operator()(const CBlob &blob) const
      {
        return blob.maxy - blob.miny;
      }
      const char *GetNom() const
      {
        return "CBlobGetDiffY";
      }
    };

    // Class to calculate the P,Q moment of a blob
    class CBlobGetMoment: public COperadorBlob
    {
    public:
      CBlobGetMoment()
      {
        m_p = m_q = 0;
      }

      CBlobGetMoment( int p, int q )
      {
        m_p = p;
        m_q = q;
      };
      double operator()(const CBlob &blob) const;
      const char *GetNom() const
      {
        return "CBlobGetMoment";
      }

    private:
      int m_p, m_q;
    };

    // Class to calculate the convex hull perimeter of a blob
    class CBlobGetHullPerimeter: public COperadorBlob
    {
    public:
      double operator()(const CBlob &blob) const;
      const char *GetNom() const
      {
        return "CBlobGetHullPerimeter";
      }
    };

    // Class to calculate the convex hull area of a blob
    class CBlobGetHullArea: public COperadorBlob
    {
    public:
      double operator()(const CBlob &blob) const;
      const char *GetNom() const
      {
        return "CBlobGetHullArea";
      }
    };

    // Class to calculate the minimum x on the minimum y
    class CBlobGetMinXatMinY: public COperadorBlob
    {
    public:
      double operator()(const CBlob &blob) const;
      const char *GetNom() const
      {
        return "CBlobGetMinXatMinY";
      }
    };

    // Class to calculate the minimum y on the maximum x
    class CBlobGetMinYatMaxX: public COperadorBlob
    {
    public:
      double operator()(const CBlob &blob) const;
      const char *GetNom() const
      {
        return "CBlobGetMinYatMaxX";
      }
    };

    // Class to calculate the maximum x on the maximum y
    class CBlobGetMaxXatMaxY: public COperadorBlob
    {
    public:
      double operator()(const CBlob &blob) const;
      const char *GetNom() const
      {
        return "CBlobGetMaxXatMaxY";
      }
    };

    // Class to calculate the maximum y on the minimum y
    class CBlobGetMaxYatMinX: public COperadorBlob
    {
    public:
      double operator()(const CBlob &blob) const;
      const char *GetNom() const
      {
        return "CBlobGetMaxYatMinX";
      }
    };

    // Class to get the minimum x
    class CBlobGetMinX: public COperadorBlob
    {
    public:
      double operator()(const CBlob &blob) const
      {
        return blob.MinX();
      }
      const char *GetNom() const
      {
        return "CBlobGetMinX";
      }
    };

    // Class to get the maximum x
    class CBlobGetMaxX: public COperadorBlob
    {
    public:
      double operator()(const CBlob &blob) const
      {
        return blob.MaxX();
      }
      const char *GetNom() const
      {
        return "CBlobGetMaxX";
      }
    };

    // Class to get the minimum y
    class CBlobGetMinY: public COperadorBlob
    {
    public:
      double operator()(const CBlob &blob) const
      {
        return blob.MinY();
      }
      const char *GetNom() const
      {
        return "CBlobGetMinY";
      }
    };

    // Class to get the maximum y
    class CBlobGetMaxY: public COperadorBlob
    {
    public:
      double operator()(const CBlob &blob) const
      {
        return blob.MaxY();
      }
      const char *GetNom() const
      {
        return "CBlobGetMax";
      }
    };

    // Class to calculate the elongation of the blob
    class CBlobGetElongation: public COperadorBlob
    {
    public:
      double operator()(const CBlob &blob) const;
      const char *GetNom() const
      {
        return "CBlobGetElongation";
      }
    };

    // Class to calculate the roughness of the blob
    class CBlobGetRoughness: public COperadorBlob
    {
    public:
      double operator()(const CBlob &blob) const;
      const char *GetNom() const
      {
        return "CBlobGetRoughness";
      }
    };

    // Class to calculate the euclidean distance between the center of a blob and a given point
    class CBlobGetDistanceFromPoint: public COperadorBlob
    {
    public:
      //! Standard constructor (distance to point 0,0)
      CBlobGetDistanceFromPoint()
      {
        m_x = m_y = 0.0;
      }

      CBlobGetDistanceFromPoint( const double x, const double y )
      {
        m_x = x;
        m_y = y;
      }

      double operator()(const CBlob &blob) const;
      const char *GetNom() const
      {
        return "CBlobGetDistanceFromPoint";
      }

    private:
      double m_x, m_y;
    };

    // Class to get the number of extern pixels of a blob
    class CBlobGetExternPerimeter: public COperadorBlob
    {
    public:
        double operator()(const CBlob &blob) const
        {
            return blob.ExternPerimeter();
        }

        const char *GetNom() const
        {
            return "CBlobGetExternPerimeter";
        }
    };

    // Class to calculate the ratio between the perimeter and the number of extern pixels
    class CBlobGetExternPerimeterRatio: public COperadorBlob
    {
    public:
        double operator()(const CBlob &blob) const
        {
            if( blob.Perimeter() != 0 )
              return blob.ExternPerimeter() / blob.Perimeter();
            else
              return blob.ExternPerimeter();
        }

        const char *GetNom() const
        {
            return "CBlobGetExternPerimeterRatio";
        }
    };

    // Class to calculate the ratio between the perimeter and the number of extern pixels
    class CBlobGetExternHullPerimeterRatio: public COperadorBlob
    {
    public:
        double operator()(const CBlob &blob) const
        {
            CBlobGetHullPerimeter getHullPerimeter;
            double hullPerimeter;

            if( (hullPerimeter = getHullPerimeter( blob ) ) != 0 )
                return blob.ExternPerimeter() / hullPerimeter;
            else
                return blob.ExternPerimeter();
        }

        const char *GetNom() const {
            return "CBlobGetExternHullPerimeterRatio";
        }
    };

    // Class to calculate the center in the X direction
    class CBlobGetXCenter: public COperadorBlob
    {
    public:
        double operator()(const CBlob &blob) const {
            return blob.MinX() + (( blob.MaxX() - blob.MinX() ) / 2.0);
        }

        const char *GetNom() const {
            return "CBlobGetXCenter";
        }
    };

    // Class to calculate the center in the Y direction
    class CBlobGetYCenter: public COperadorBlob
    {
    public:
        double operator()(const CBlob &blob) const {
            return blob.MinY() + (( blob.MaxY() - blob.MinY() ) / 2.0);
        }

        const char *GetNom() const {
            return "CBlobGetYCenter";
        }
    };

    // Class to calculate the length of the major axis of the ellipse that fits the blob edges
    class CBlobGetMajorAxisLength: public COperadorBlob
    {
    public:
        double operator()(const CBlob &blob) const {
            CvBox2D elipse = blob.GetEllipse();
            return elipse.size.width;
        }

        const char *GetNom() const {
            return "CBlobGetMajorAxisLength";
        }
    };

    class CBlobGetAreaElipseRatio: public COperadorBlob
    {
    public:
        double operator()(const CBlob &blob) const
        {
            if( blob.Area()==0.0 ) return 0.0;

            CvBox2D elipse = blob.GetEllipse();
            double ratioAreaElipseAreaTaca = ((elipse.size.width/2.0)*(elipse.size.height/2.0)*CV_PI)/blob.Area();
            return ratioAreaElipseAreaTaca;
        }

        const char *GetNom() const {
            return "CBlobGetAreaElipseRatio";
        }
    };

    // Class to calculate the length of the minor axis of the ellipse that fits the blob edges
    class CBlobGetMinorAxisLength: public COperadorBlob
    {
    public:
        double operator()(const CBlob &blob) const {
            CvBox2D elipse = blob.GetEllipse();
            return elipse.size.height;
        }

        const char *GetNom() const {
            return "CBlobGetMinorAxisLength";
        }
    };

    // Class to calculate the orientation of the ellipse that fits the blob edges in radians
    class CBlobGetOrientation: public COperadorBlob
    {
    public:
        double operator()(const CBlob &blob) const {
            CvBox2D elipse = blob.GetEllipse();

            if( elipse.angle > 180.0 )
              return (( elipse.angle - 180.0 )* DEGREE2RAD);
            else
              return ( elipse.angle * DEGREE2RAD);
        }

        const char *GetNom() const {
            return "CBlobGetOrientation";
        }
    };

    // Class to calculate the cosinus of the orientation of the ellipse that fits the blob edges
    class CBlobGetOrientationCos: public COperadorBlob
    {
    public:
        double operator()(const CBlob &blob) const {
            CBlobGetOrientation getOrientation;
            return fabs( cos( getOrientation(blob) ));
        }

        const char *GetNom() const {
            return "CBlobGetOrientationCos";
        }
    };


    // Class to calculate the ratio between both axes of the ellipse
    class CBlobGetAxisRatio: public COperadorBlob
    {
    public:
        double operator()(const CBlob &blob) const {
            CvBox2D elipse = blob.GetEllipse();
            return elipse.size.height / elipse.size.width;
        }

        const char *GetNom() const {
            return "CBlobGetAxisRatio";
        }
    };

    // Class to calculate whether a point is inside a blob
    class CBlobGetXYInside: public COperadorBlob
    {
    public:
        CBlobGetXYInside() {
            m_p = cvPoint(0,0);
        }

        CBlobGetXYInside( CvPoint p ) {
            m_p = p;
        };

        double operator()(const CBlob &blob) const;

        const char *GetNom() const {
            return "CBlobGetXYInside";
        }

    private:
        CvPoint m_p;
    };

    typedef std::vector<CBlob*>	blob_vector;

    class CBlobResult
    {
    public:
        CBlobResult();
        CBlobResult(IplImage *source, IplImage *mask, int threshold, bool findmoments);
        CBlobResult( const CBlobResult &source );
        virtual ~CBlobResult();

        // Assigment operator
        CBlobResult& operator=(const CBlobResult& source);
        CBlobResult operator+( const CBlobResult& source );

        void AddBlob( CBlob *blob );

    #ifdef MATRIXCV_ACTIU
        double_vector GetResult( funcio_calculBlob *evaluador ) const;
    #endif

        double_stl_vector GetSTLResult( funcio_calculBlob *evaluador ) const;

        double GetNumber( int indexblob, funcio_calculBlob *evaluador ) const;

        void Filter(CBlobResult &dst,
                    int filterAction, funcio_calculBlob *evaluador,
                    int condition, double lowLimit, double highLimit = 0 );

        // Sorts the blobs of the class acording to some criteria and returns the n-th blob
        void GetNthBlob( funcio_calculBlob *criteri, int nBlob, CBlob &dst ) const;

        // Gets the n-th blob of the class ( without sorting )
        CBlob GetBlob(int indexblob) const;
        CBlob *GetBlob(int indexblob);

        //! Elimina tots els blobs de l'objecte
        //! Clears all the blobs of the class
        void ClearBlobs();

        //! Escriu els blobs a un fitxer
        //! Prints some features of all the blobs in a file
        void PrintBlobs( char *nom_fitxer ) const;

        // Gets the total number of blobs
        int GetNumBlobs() const
        {
            return(m_blobs.size());
        }
    private:
        void RaiseError(const int errorCode) const;

    protected:
        blob_vector		m_blobs;
    };

    bool BlobAnalysis(IplImage* inputImage, uchar threshold, IplImage* maskImage, bool borderColor, bool findmoments, blob_vector &RegionData );
    void Subsume(blob_vector &RegionData, int, int*, CBlob*, CBlob*, bool, int, int );
    int *NewSubsume(int *SubSumedRegion, int elems_inbuffer);
    double GetExternPerimeter( int start, int end, int row, int width, int height, IplImage *maskImage );
}

#endif // AMBP_OBJECTEXTRACTION_H
