#include "ambp_objectextraction.h"

namespace AMBP_OBJECT
{
    CBlob::CBlob()
    {
      etiqueta = -1;
      exterior = 0;
      area = 0.0f;
      perimeter = 0.0f;
      parent = -1;
      minx = LONG_MAX;
      maxx = 0;
      miny = LONG_MAX;
      maxy = 0;
      sumx = 0;
      sumy = 0;
      sumxx = 0;
      sumyy = 0;
      sumxy = 0;
      mean = 0;
      stddev = 0;
      externPerimeter = 0;

      m_storage = cvCreateMemStorage(0);
      edges = cvCreateSeq( CV_SEQ_KIND_GENERIC|CV_32SC2,
        sizeof(CvContour),
        sizeof(CvPoint),m_storage);
    }

    CBlob::CBlob( const CBlob &src )
    {
      etiqueta = src.etiqueta;
      exterior = src.exterior;
      area = src.Area();
      perimeter = src.Perimeter();
      parent = src.parent;
      minx = src.minx;
      maxx = src.maxx;
      miny = src.miny;
      maxy = src.maxy;
      sumx = src.sumx;
      sumy = src.sumy;
      sumxx = src.sumxx;
      sumyy = src.sumyy;
      sumxy = src.sumxy;
      mean = src.mean;
      stddev = src.stddev;
      externPerimeter = src.externPerimeter;

      CvSeqReader reader;
      CvSeqWriter writer;
      CvPoint edgeactual;

      m_storage = cvCreateMemStorage(0);
      edges = cvCreateSeq( CV_SEQ_KIND_GENERIC|CV_32SC2,
        sizeof(CvContour),
        sizeof(CvPoint),m_storage);

      cvStartReadSeq( src.Edges(), &reader);
      cvStartAppendToSeq( edges, &writer );

      for( int i=0; i< src.Edges()->total; i++)
      {
        CV_READ_SEQ_ELEM( edgeactual ,reader);
        CV_WRITE_SEQ_ELEM( edgeactual , writer );
      }

      cvEndWriteSeq( &writer );
    }

    CBlob::CBlob( const CBlob *src )
    {
      etiqueta = src->etiqueta;
      exterior = src->exterior;
      area = src->Area();
      perimeter = src->Perimeter();
      parent = src->parent;
      minx = src->minx;
      maxx = src->maxx;
      miny = src->miny;
      maxy = src->maxy;
      sumx = src->sumx;
      sumy = src->sumy;
      sumxx = src->sumxx;
      sumyy = src->sumyy;
      sumxy = src->sumxy;
      mean = src->mean;
      stddev = src->stddev;
      externPerimeter = src->externPerimeter;

      CvSeqReader reader;
      CvSeqWriter writer;
      CvPoint edgeactual;

      m_storage = cvCreateMemStorage(0);
      edges = cvCreateSeq( CV_SEQ_KIND_GENERIC|CV_32SC2,
        sizeof(CvContour),
        sizeof(CvPoint),m_storage);

      cvStartReadSeq( src->Edges(), &reader);
      cvStartAppendToSeq( edges, &writer );

      for( int i=0; i< src->Edges()->total; i++)
      {
        CV_READ_SEQ_ELEM( edgeactual ,reader);
        CV_WRITE_SEQ_ELEM( edgeactual , writer );
      }

      cvEndWriteSeq( &writer );
    }

    CBlob::~CBlob()
    {
      cvClearSeq(edges);
      cvReleaseMemStorage( &m_storage );
    }

    CBlob& CBlob::operator=(const CBlob &src )
    {
      if (this != &src)
      {
        cvClearSeq(edges);
        cvReleaseMemStorage( &m_storage );

        m_storage = cvCreateMemStorage(0);
        edges = cvCreateSeq( CV_SEQ_KIND_GENERIC|CV_32SC2,
          sizeof(CvContour),
          sizeof(CvPoint),m_storage);

        etiqueta = src.etiqueta;
        exterior = src.exterior;
        area = src.Area();
        perimeter = src.Perimeter();
        parent = src.parent;
        minx = src.minx;
        maxx = src.maxx;
        miny = src.miny;
        maxy = src.maxy;
        sumx = src.sumx;
        sumy = src.sumy;
        sumxx = src.sumxx;
        sumyy = src.sumyy;
        sumxy = src.sumxy;
        mean = src.mean;
        stddev = src.stddev;
        externPerimeter = src.externPerimeter;

        CvSeqReader reader;
        CvSeqWriter writer;
        CvPoint edgeactual;

        cvStartReadSeq( src.Edges(), &reader);
        cvStartAppendToSeq( edges, &writer );

        for( int i=0; i< src.Edges()->total; i++)
        {
          CV_READ_SEQ_ELEM( edgeactual ,reader);
          CV_WRITE_SEQ_ELEM( edgeactual , writer );
        }

        cvEndWriteSeq( &writer );
      }
      return *this;
    }

    void CBlob::FillBlob( IplImage *imatge, CvScalar color, int offsetX /*=0*/, int offsetY /*=0*/) const
    {
      if( edges == NULL || edges->total == 0 ) return;

      CvPoint edgeactual, pt1, pt2;
      CvSeqReader reader;
      vectorPunts vectorEdges = vectorPunts( edges->total );
      vectorPunts::iterator itEdges, itEdgesSeguent;
      bool dinsBlob;
      int yActual;

      cvStartReadSeq( edges, &reader);
      itEdges = vectorEdges.begin();
      while( itEdges != vectorEdges.end() )
      {
        CV_READ_SEQ_ELEM( edgeactual ,reader);
        *itEdges = edgeactual;
        itEdges++;
      }

      std::sort( vectorEdges.begin(), vectorEdges.end(), comparaCvPoint() );

      itEdges = vectorEdges.begin();
      itEdgesSeguent = vectorEdges.begin() + 1;
      dinsBlob = true;
      while( itEdges != (vectorEdges.end() - 1))
      {
        yActual = (*itEdges).y;

        if( ( (*itEdges).x != (*itEdgesSeguent).x ) &&
          ( (*itEdgesSeguent).y == yActual )
          )
        {
          if( dinsBlob )
          {
            pt1 = *itEdges;
            pt1.x += offsetX;
            pt1.y += offsetY;

            pt2 = *itEdgesSeguent;
            pt2.x += offsetX;
            pt2.y += offsetY;

            cvLine( imatge, pt1, pt2, color );
          }
          dinsBlob =! dinsBlob;
        }

        itEdges++;
        itEdgesSeguent++;
        if( (*itEdges).y != yActual ) dinsBlob = true;
      }
      vectorEdges.clear();
    }

    void CBlob::CopyEdges( CBlob &destination ) const
    {
      CvSeqReader reader;
      CvSeqWriter writer;
      CvPoint edgeactual;

      cvStartReadSeq( edges, &reader);
      cvStartAppendToSeq( destination.Edges(), &writer );

      for( int i=0; i<edges->total; i++)
      {
        CV_READ_SEQ_ELEM( edgeactual ,reader);
        CV_WRITE_SEQ_ELEM( edgeactual , writer );
      }

      cvEndWriteSeq( &writer );
    }

    void CBlob::ClearEdges()
    {
      cvClearSeq( edges );
    }

    bool CBlob::GetConvexHull( CvSeq **dst ) const
    {
      if( edges != NULL && edges->total > 0)
      {
        *dst = cvConvexHull2( edges, 0, CV_CLOCKWISE, 0 );
        return true;
      }
      return false;
    }

    CvBox2D CBlob::GetEllipse() const
    {
      CvBox2D elipse;
      // necessitem 6 punts per calcular l'elipse
      if( edges != NULL && edges->total > 6)
      {
        elipse = cvFitEllipse2( edges );
      }
      else
      {
        elipse.center.x = 0.0;
        elipse.center.y = 0.0;
        elipse.size.width = 0.0;
        elipse.size.height = 0.0;
        elipse.angle = 0.0;
      }
      return elipse;
    }

    double CBlobGetMoment::operator()(const CBlob &blob) const
    {
      //Moment 00
      if((m_p==0) && (m_q==0))
        return blob.Area();

      //Moment 10
      if((m_p==1) && (m_q==0))
        return blob.SumX();

      //Moment 01
      if((m_p==0) && (m_q==1))
        return blob.SumY();

      //Moment 20
      if((m_p==2) && (m_q==0))
        return blob.SumXX();

      //Moment 02
      if((m_p==0) && (m_q==2))
        return blob.SumYY();

      return 0;
    }

    double CBlobGetHullPerimeter::operator()(const CBlob &blob) const
    {
      if(blob.Edges() != NULL && blob.Edges()->total > 0)
      {
        CvSeq *hull = cvConvexHull2( blob.Edges(), 0, CV_CLOCKWISE, 1 );
        return fabs(cvArcLength(hull,CV_WHOLE_SEQ,1));
      }
      return blob.Perimeter();
    }

    double CBlobGetHullArea::operator()(const CBlob &blob) const
    {
      if(blob.Edges() != NULL && blob.Edges()->total > 0)
      {
        CvSeq *hull = cvConvexHull2( blob.Edges(), 0, CV_CLOCKWISE, 1 );
        return fabs(cvContourArea(hull));
      }
      return blob.Perimeter();
    }

    double CBlobGetMinXatMinY::operator()(const CBlob &blob) const
    {
      double MinX_at_MinY = LONG_MAX;

      CvSeqReader reader;
      CvPoint edgeactual;

      cvStartReadSeq(blob.Edges(),&reader);

      for(int j=0;j<blob.Edges()->total;j++)
      {
        CV_READ_SEQ_ELEM(edgeactual,reader);
        if( (edgeactual.y == blob.MinY()) && (edgeactual.x < MinX_at_MinY) )
        {
          MinX_at_MinY = edgeactual.x;
        }
      }

      return MinX_at_MinY;
    }

    double CBlobGetMinYatMaxX::operator()(const CBlob &blob) const
    {
      double MinY_at_MaxX = LONG_MAX;

      CvSeqReader reader;
      CvPoint edgeactual;

      cvStartReadSeq(blob.Edges(),&reader);

      for(int j=0;j<blob.Edges()->total;j++)
      {
        CV_READ_SEQ_ELEM(edgeactual,reader);
        if( (edgeactual.x == blob.MaxX()) && (edgeactual.y < MinY_at_MaxX) )
        {
          MinY_at_MaxX = edgeactual.y;
        }
      }

      return MinY_at_MaxX;
    }

    double CBlobGetMaxXatMaxY::operator()(const CBlob &blob) const
    {
      double MaxX_at_MaxY = LONG_MIN;

      CvSeqReader reader;
      CvPoint edgeactual;

      cvStartReadSeq(blob.Edges(),&reader);

      for(int j=0;j<blob.Edges()->total;j++)
      {
        CV_READ_SEQ_ELEM(edgeactual,reader);
        if( (edgeactual.y == blob.MaxY()) && (edgeactual.x > MaxX_at_MaxY) )
        {
          MaxX_at_MaxY = edgeactual.x;
        }
      }

      return MaxX_at_MaxY;
    }

    double CBlobGetMaxYatMinX::operator()(const CBlob &blob) const
    {
      double MaxY_at_MinX = LONG_MIN;

      CvSeqReader reader;
      CvPoint edgeactual;

      cvStartReadSeq(blob.Edges(),&reader);

      for(int j=0;j<blob.Edges()->total;j++)
      {
        CV_READ_SEQ_ELEM(edgeactual,reader);
        if( (edgeactual.x == blob.MinY()) && (edgeactual.y > MaxY_at_MinX) )
        {
          MaxY_at_MinX = edgeactual.y;
        }
      }

      return MaxY_at_MinX;
    }

    double CBlobGetElongation::operator()(const CBlob &blob) const
    {
      double ampladaC,longitudC,amplada,longitud;

      ampladaC=(double) (blob.Perimeter()+sqrt(pow(blob.Perimeter(),2)-16*blob.Area()))/4;
      if(ampladaC<=0.0) return 0;
      longitudC=(double) blob.Area()/ampladaC;

      longitud=MAX( longitudC , ampladaC );
      amplada=MIN( longitudC , ampladaC );

      return (double) longitud/amplada;
    }

    double CBlobGetCompactness::operator()(const CBlob &blob) const
    {
      if( blob.Area() != 0.0 )
        return (double) pow(blob.Perimeter(),2)/(4*CV_PI*blob.Area());
      else
        return 0.0;
    }

    double CBlobGetRoughness::operator()(const CBlob &blob) const
    {
      CBlobGetHullPerimeter getHullPerimeter = CBlobGetHullPerimeter();

      double hullPerimeter = getHullPerimeter(blob);

      if( hullPerimeter != 0.0 )
        return blob.Perimeter() / hullPerimeter;

      return 0.0;
    }

    double CBlobGetLength::operator()(const CBlob &blob) const
    {
      double ampladaC,longitudC;
      double tmp;

      tmp = blob.Perimeter()*blob.Perimeter() - 16*blob.Area();

      if( tmp > 0.0 )
        ampladaC = (double) (blob.Perimeter()+sqrt(tmp))/4;
      else
        ampladaC = (double) (blob.Perimeter())/4;

      if(ampladaC<=0.0) return 0;
      longitudC=(double) blob.Area()/ampladaC;

      return MAX( longitudC , ampladaC );
    }

    double CBlobGetBreadth::operator()(const CBlob &blob) const
    {
      double ampladaC,longitudC;
      double tmp;

      tmp = blob.Perimeter()*blob.Perimeter() - 16*blob.Area();

      if( tmp > 0.0 )
        ampladaC = (double) (blob.Perimeter()+sqrt(tmp))/4;
      else
        ampladaC = (double) (blob.Perimeter())/4;

      if(ampladaC<=0.0) return 0;
      longitudC = (double) blob.Area()/ampladaC;

      return MIN( longitudC , ampladaC );
    }

    double CBlobGetDistanceFromPoint::operator()(const CBlob &blob) const
    {
      double xmitjana, ymitjana;
      CBlobGetXCenter getXCenter;
      CBlobGetYCenter getYCenter;

      xmitjana = m_x - getXCenter( blob );
      ymitjana = m_y - getYCenter( blob );

      return sqrt((xmitjana*xmitjana)+(ymitjana*ymitjana));
    }

    double CBlobGetXYInside::operator()(const CBlob &blob) const
    {
      if( blob.Edges() == NULL || blob.Edges()->total == 0 ) return 0.0;

      CvSeqReader reader;
      CBlob::vectorPunts vectorEdges;
      CBlob::vectorPunts::iterator itEdges, itEdgesSeguent;
      CvPoint edgeactual;
      bool dinsBlob;

      cvStartReadSeq( blob.Edges(), &reader);

      for( int i=0; i< blob.Edges()->total; i++)
      {
        CV_READ_SEQ_ELEM( edgeactual ,reader );
        if( edgeactual.y == m_p.y )
          vectorEdges.push_back( edgeactual );
      }

      if( vectorEdges.size() == 0 ) return 0.0;

      std::sort( vectorEdges.begin(), vectorEdges.end(), CBlob::comparaCvPoint() );

      itEdges = vectorEdges.begin();
      itEdgesSeguent = vectorEdges.begin() + 1;
      dinsBlob = true;

      while( itEdges != (vectorEdges.end() - 1) )
      {
        if( (*itEdges).x <= m_p.x && (*itEdgesSeguent).x >= m_p.x && dinsBlob )
        {
          vectorEdges.clear();
          return 1.0;
        }

        itEdges++;
        itEdgesSeguent++;
        dinsBlob = !dinsBlob;
      }

      vectorEdges.clear();
      return 0.0;
    }

    //////////////////////////////////////////////
    CBlobResult::CBlobResult()
    {
      m_blobs = blob_vector();
    }

    CBlobResult::CBlobResult(IplImage *source, IplImage *mask, int threshold, bool findmoments)
    {
      bool success;

      try
      {
        success = BlobAnalysis(source,(uchar)threshold,mask,true,findmoments, m_blobs );
      }
      catch(...)
      {
        success = false;
      }

      if( !success ) throw EXCEPCIO_CALCUL_BLOBS;
    }

    ////////////////////
    CBlobResult::CBlobResult( const CBlobResult &source )
    {
      m_blobs = blob_vector( source.GetNumBlobs() );
      m_blobs = blob_vector( source.GetNumBlobs() );
      blob_vector::const_iterator pBlobsSrc = source.m_blobs.begin();
      blob_vector::iterator pBlobsDst = m_blobs.begin();

      while( pBlobsSrc != source.m_blobs.end() )
      {
        *pBlobsDst = new CBlob(**pBlobsSrc);
        pBlobsSrc++;
        pBlobsDst++;
      }
    }

    CBlobResult::~CBlobResult()
    {
      ClearBlobs();
    }

    CBlobResult& CBlobResult::operator=(const CBlobResult& source)
    {
      if (this != &source)
      {
        for( int i = 0; i < GetNumBlobs(); i++ )
        {
          delete m_blobs[i];
        }
        m_blobs.clear();
        m_blobs = blob_vector( source.GetNumBlobs() );
        blob_vector::const_iterator pBlobsSrc = source.m_blobs.begin();
        blob_vector::iterator pBlobsDst = m_blobs.begin();

        while( pBlobsSrc != source.m_blobs.end() )
        {
          *pBlobsDst = new CBlob(**pBlobsSrc);
          pBlobsSrc++;
          pBlobsDst++;
        }
      }
      return *this;
    }

    CBlobResult CBlobResult::operator+( const CBlobResult& source )
    {
      CBlobResult resultat( *this );

      resultat.m_blobs.resize( resultat.GetNumBlobs() + source.GetNumBlobs() );

      blob_vector::const_iterator pBlobsSrc = source.m_blobs.begin();
      blob_vector::iterator pBlobsDst = resultat.m_blobs.end();

      while( pBlobsSrc != source.m_blobs.end() )
      {
        pBlobsDst--;
        *pBlobsDst = new CBlob(**pBlobsSrc);
        pBlobsSrc++;
      }

      return resultat;
    }

    void CBlobResult::AddBlob( CBlob *blob )
    {
      if( blob != NULL )
        m_blobs.push_back( new CBlob( blob ) );
    }

    #ifdef MATRIXCV_ACTIU
    double_vector CBlobResult::GetResult( funcio_calculBlob *evaluador ) const
    {
      if( GetNumBlobs() <= 0 )
      {
        return double_vector();
      }

      double_vector result = double_vector( GetNumBlobs() );
      double_vector::iterator itResult = result.GetIterator();
      blob_vector::const_iterator itBlobs = m_blobs.begin();

      while( itBlobs != m_blobs.end() )
      {
        *itResult = (*evaluador)(**itBlobs);
        itBlobs++;
        itResult++;
      }
      return result;
    }
    #endif

    double_stl_vector CBlobResult::GetSTLResult( funcio_calculBlob *evaluador ) const
    {
      if( GetNumBlobs() <= 0 )
      {
        return double_stl_vector();
      }

      double_stl_vector result = double_stl_vector( GetNumBlobs() );
      double_stl_vector::iterator itResult = result.begin();
      blob_vector::const_iterator itBlobs = m_blobs.begin();

      while( itBlobs != m_blobs.end() )
      {
        *itResult = (*evaluador)(**itBlobs);
        itBlobs++;
        itResult++;
      }
      return result;
    }

    double CBlobResult::GetNumber( int indexBlob, funcio_calculBlob *evaluador ) const
    {
      if( indexBlob < 0 || indexBlob >= GetNumBlobs() )
        RaiseError( EXCEPTION_BLOB_OUT_OF_BOUNDS );
      return (*evaluador)( *m_blobs[indexBlob] );
    }

    void CBlobResult::Filter(CBlobResult &dst,
      int filterAction,
      funcio_calculBlob *evaluador,
      int condition,
      double lowLimit, double highLimit /*=0*/)

    {
      int i, numBlobs;
      bool resultavaluacio;
      double_stl_vector avaluacioBlobs;
      double_stl_vector::iterator itavaluacioBlobs;

      if( GetNumBlobs() <= 0 ) return;
      if( !evaluador ) return;

      avaluacioBlobs = GetSTLResult(evaluador);
      itavaluacioBlobs = avaluacioBlobs.begin();
      numBlobs = GetNumBlobs();
      switch(condition)
      {
      case B_EQUAL:
        for(i=0;i<numBlobs;i++, itavaluacioBlobs++)
        {
          resultavaluacio= *itavaluacioBlobs == lowLimit;
          if( ( resultavaluacio && filterAction == B_INCLUDE ) ||
            ( !resultavaluacio && filterAction == B_EXCLUDE ))
          {
            dst.m_blobs.push_back( new CBlob( GetBlob( i ) ));
          }
        }
        break;
      case B_NOT_EQUAL:
        for(i=0;i<numBlobs;i++, itavaluacioBlobs++)
        {
          resultavaluacio = *itavaluacioBlobs != lowLimit;
          if( ( resultavaluacio && filterAction == B_INCLUDE ) ||
            ( !resultavaluacio && filterAction == B_EXCLUDE ))
          {
            dst.m_blobs.push_back( new CBlob( GetBlob( i ) ));
          }
        }
        break;
      case B_GREATER:
        for(i=0;i<numBlobs;i++, itavaluacioBlobs++)
        {
          resultavaluacio= *itavaluacioBlobs > lowLimit;
          if( ( resultavaluacio && filterAction == B_INCLUDE ) ||
            ( !resultavaluacio && filterAction == B_EXCLUDE ))
          {
            dst.m_blobs.push_back( new CBlob( GetBlob( i ) ));
          }
        }
        break;
      case B_LESS:
        for(i=0;i<numBlobs;i++, itavaluacioBlobs++)
        {
          resultavaluacio= *itavaluacioBlobs < lowLimit;
          if( ( resultavaluacio && filterAction == B_INCLUDE ) ||
            ( !resultavaluacio && filterAction == B_EXCLUDE ))
          {
            dst.m_blobs.push_back( new CBlob( GetBlob( i ) ));
          }
        }
        break;
      case B_GREATER_OR_EQUAL:
        for(i=0;i<numBlobs;i++, itavaluacioBlobs++)
        {
          resultavaluacio= *itavaluacioBlobs>= lowLimit;
          if( ( resultavaluacio && filterAction == B_INCLUDE ) ||
            ( !resultavaluacio && filterAction == B_EXCLUDE ))
          {
            dst.m_blobs.push_back( new CBlob( GetBlob( i ) ));
          }
        }
        break;
      case B_LESS_OR_EQUAL:
        for(i=0;i<numBlobs;i++, itavaluacioBlobs++)
        {
          resultavaluacio= *itavaluacioBlobs <= lowLimit;
          if( ( resultavaluacio && filterAction == B_INCLUDE ) ||
            ( !resultavaluacio && filterAction == B_EXCLUDE ))
          {
            dst.m_blobs.push_back( new CBlob( GetBlob( i ) ));
          }
        }
        break;
      case B_INSIDE:
        for(i=0;i<numBlobs;i++, itavaluacioBlobs++)
        {
          resultavaluacio=( *itavaluacioBlobs >= lowLimit) && ( *itavaluacioBlobs <= highLimit);
          if( ( resultavaluacio && filterAction == B_INCLUDE ) ||
            ( !resultavaluacio && filterAction == B_EXCLUDE ))
          {
            dst.m_blobs.push_back( new CBlob( GetBlob( i ) ));
          }
        }
        break;
      case B_OUTSIDE:
        for(i=0;i<numBlobs;i++, itavaluacioBlobs++)
        {
          resultavaluacio=( *itavaluacioBlobs < lowLimit) || ( *itavaluacioBlobs > highLimit);
          if( ( resultavaluacio && filterAction == B_INCLUDE ) ||
            ( !resultavaluacio && filterAction == B_EXCLUDE ))
          {
            dst.m_blobs.push_back( new CBlob( GetBlob( i ) ));
          }
        }
        break;
      }

      if( &dst == this )
      {
        blob_vector::iterator itBlobs = m_blobs.begin();
        for( int i = 0; i < numBlobs; i++ )
        {
          delete *itBlobs;
          itBlobs++;
        }
        m_blobs.erase( m_blobs.begin(), itBlobs );
      }
    }

    CBlob CBlobResult::GetBlob(int indexblob) const
    {
      if( indexblob < 0 || indexblob >= GetNumBlobs() )
        RaiseError( EXCEPTION_BLOB_OUT_OF_BOUNDS );

      return *m_blobs[indexblob];
    }

    CBlob *CBlobResult::GetBlob(int indexblob)
    {
      if( indexblob < 0 || indexblob >= GetNumBlobs() )
        RaiseError( EXCEPTION_BLOB_OUT_OF_BOUNDS );

      return m_blobs[indexblob];
    }

    void CBlobResult::GetNthBlob( funcio_calculBlob *criteri, int nBlob, CBlob &dst ) const
    {
      if( nBlob < 0 || nBlob >= GetNumBlobs() )
      {
        dst = CBlob();
        return;
      }

      double_stl_vector avaluacioBlobs, avaluacioBlobsOrdenat;
      double valorEnessim;

      avaluacioBlobs = GetSTLResult(criteri);
      avaluacioBlobsOrdenat = double_stl_vector( GetNumBlobs() );

      std::partial_sort_copy( avaluacioBlobs.begin(),
        avaluacioBlobs.end(),
        avaluacioBlobsOrdenat.begin(),
        avaluacioBlobsOrdenat.end(),
        std::greater<double>() );

      valorEnessim = avaluacioBlobsOrdenat[nBlob];

      double_stl_vector::const_iterator itAvaluacio = avaluacioBlobs.begin();

      bool trobatBlob = false;
      int indexBlob = 0;
      while( itAvaluacio != avaluacioBlobs.end() && !trobatBlob )
      {
        if( *itAvaluacio == valorEnessim )
        {
          trobatBlob = true;
          dst = CBlob( GetBlob(indexBlob));
        }
        itAvaluacio++;
        indexBlob++;
      }
    }

    void CBlobResult::ClearBlobs()
    {
      blob_vector::iterator itBlobs = m_blobs.begin();
      while( itBlobs != m_blobs.end() )
      {
        delete *itBlobs;
        itBlobs++;
      }

      m_blobs.clear();
    }

    void CBlobResult::RaiseError(const int errorCode) const
    {
      throw errorCode;
    }

    void CBlobResult::PrintBlobs( char *nom_fitxer ) const
    {
      double_stl_vector area, exterior, mitjana, compacitat, longitud,
        externPerimeter, perimetreConvex, perimetre;
      int i;
      FILE *fitxer_sortida;

      area      = GetSTLResult( CBlobGetArea());
      perimetre = GetSTLResult( CBlobGetPerimeter());
      exterior  = GetSTLResult( CBlobGetExterior());
      mitjana   = GetSTLResult( CBlobGetMean());
      compacitat = GetSTLResult(CBlobGetCompactness());
      longitud  = GetSTLResult( CBlobGetLength());
      externPerimeter = GetSTLResult( CBlobGetExternPerimeter());
      perimetreConvex = GetSTLResult( CBlobGetHullPerimeter());

      fitxer_sortida = fopen( nom_fitxer, "w" );

      for(i=0; i<GetNumBlobs(); i++)
      {
        fprintf( fitxer_sortida, "blob %d ->\t a=%7.0f\t p=%8.2f (%8.2f extern)\t pconvex=%8.2f\t ext=%.0f\t m=%7.2f\t c=%3.2f\t l=%8.2f\n",
          i, area[i], perimetre[i], externPerimeter[i], perimetreConvex[i], exterior[i], mitjana[i], compacitat[i], longitud[i] );
      }

      fclose( fitxer_sortida );
    }

    /////////////////////////////////////////////////////////////////////////////////
    bool BlobAnalysis(	IplImage* inputImage,
      uchar threshold,
      IplImage* maskImage,
      bool borderColor,
      bool findmoments,
      blob_vector &RegionData )
    {
      // dimensions of input image taking in account the ROI
      int Cols, Rows, startCol, startRow;

      if( inputImage->roi )
      {
        CvRect imageRoi = cvGetImageROI( inputImage );
        startCol = imageRoi.x;
        startRow = imageRoi.y;
        Cols = imageRoi.width;
        Rows = imageRoi.height;
      }
      else
      {
        startCol = 0;
        startRow = 0;
        Cols = inputImage->width;
        Rows = inputImage->height;
      }

      int Trans = Cols;				// MAX trans in any row
      char* pMask = NULL;
      char* pImage;

      // Convert image array into transition array. In each row the transition array tells which columns have a color change
      int iCol,iRow,iTran, Tran;				// Data for a given run
      bool ThisCell, LastCell;                  // Contents (colors (0 or 1)) within this row
      int TransitionOffset = 0;                 // Performance booster to avoid multiplication

      // row 0 and row Rows+1 represent the border
      int i;
      int *Transition;                          // Transition Matrix

      int nombre_pixels_mascara = 0;
      //! Imatge amb el perimetre extern de cada pixel
      IplImage *imatgePerimetreExtern;

      // input images must have only 1-channel and be an image
      if( !CV_IS_IMAGE( inputImage ) || (inputImage->nChannels != 1) )
      {
        return false;
      }
      if( maskImage != NULL )
      {
        // input image and mask are a valid image
        if( !CV_IS_IMAGE( inputImage ) || !CV_IS_IMAGE( maskImage ))
          return false;

        // comprova que la màscara tingui les mateixes dimensions que la imatge
        if( inputImage->width != maskImage->width || inputImage->height != maskImage->height )
        {
          return false;
        }

        if( maskImage->nChannels != 1 )
        {
          return false;
        }
      }

      // Initialize Transition array
      Transition=new int[(Rows + 2)*(Cols + 2)];
      memset(Transition,0,(Rows + 2) * (Cols + 2)*sizeof(int));
      Transition[0] = Transition[(Rows + 1) * (Cols + 2)] = Cols + 2;

      // Start at the beginning of the image (startCol, startRow)
      pImage = inputImage->imageData + startCol - 1 + startRow * inputImage->widthStep;

      if(maskImage == NULL)
      {
        imatgePerimetreExtern = NULL;

        //Fill Transition array
        for(iRow = 1; iRow < Rows + 1; iRow++)		// Choose a row of Bordered image
        {
          TransitionOffset = iRow*(Cols + 2);       //per a que sigui paral·litzable
          iTran = 0;                                // Index into Transition array
          Tran = 0;                                 // No transitions at row start
          LastCell = borderColor;

          for(iCol = 0; iCol < Cols + 2; iCol++)	// Scan that row of Bordered image
          {
            if(iCol == 0 || iCol == Cols+1)
              ThisCell = borderColor;
            else
              ThisCell = ((unsigned char) *(pImage)) > threshold;

            if(ThisCell != LastCell)
            {
              Transition[TransitionOffset + iTran] = Tran;      // Save completed Tran
              iTran++;                                          // Prepare new index
              LastCell = ThisCell;                              // With this color
            }

            Tran++;
            pImage++;
          }

          Transition[TransitionOffset + iTran] = Tran;
          if ( (TransitionOffset + iTran + 1) < (Rows + 1)*(Cols + 2) )
          {
            Transition[TransitionOffset + iTran + 1] = -1;
          }

          //jump to next row (beginning from (startCol, startRow))
          pImage = inputImage->imageData - 1 + startCol + (iRow+startRow)*inputImage->widthStep;
        }
      }
      else
      {
        char perimeter;
        char *pPerimetre;

        imatgePerimetreExtern = cvCreateImage( cvSize(maskImage->width, maskImage->height), IPL_DEPTH_8U, 1);
        cvSetZero( imatgePerimetreExtern );

        pMask = maskImage->imageData - 1;

        //Fill Transition array
        for(iRow = 1; iRow < Rows + 1; iRow++)          // Choose a row of Bordered image
        {
          TransitionOffset = iRow*(Cols + 2);
          iTran = 0;                                    // Index into Transition array
          Tran = 0;                                     // No transitions at row start
          LastCell = borderColor;

          pPerimetre = imatgePerimetreExtern->imageData + (iRow - 1) * imatgePerimetreExtern->widthStep;

          for(iCol = 0; iCol < Cols + 2; iCol++)
          {
            if(iCol == 0 || iCol == Cols+1 || ((unsigned char) *pMask) == PIXEL_EXTERIOR)
              ThisCell = borderColor;
            else
              ThisCell = ((unsigned char) *(pImage)) > threshold;

            if(ThisCell != LastCell)
            {
              Transition[TransitionOffset + iTran] = Tran;	// Save completed Tran
              iTran++;                                      // Prepare new index
              LastCell = ThisCell;                          // With this color
            }

            if( (iCol > 0) && (iCol < Cols) )
            {
              if( *pMask == PIXEL_EXTERIOR )
              {
                *pPerimetre = 0;
              }
              else
              {
                perimeter = 0;

                if(iRow>1)
                {
                  if( *(pMask - maskImage->widthStep ) == PIXEL_EXTERIOR) perimeter++;
                }

                if( iRow < imatgePerimetreExtern->height )
                {
                  if( (iCol>0) && (*(pMask-1) == PIXEL_EXTERIOR) ) perimeter++;

                  if( ( iCol < imatgePerimetreExtern->width - 1) && (*(pMask+1) == PIXEL_EXTERIOR) ) perimeter++;
                }

                if( iRow < imatgePerimetreExtern->height - 1)
                {
                  if( (*(pMask+maskImage->widthStep) == PIXEL_EXTERIOR) ) perimeter++;
                }

                *pPerimetre = perimeter;
              }
            }

            Tran++;
            pImage++;
            pMask++;
            pPerimetre++;
          }
          Transition[TransitionOffset + iTran] = Tran;

          if ( (TransitionOffset + iTran + 1) < (Rows + 1)*(Cols + 2) )
          {
            Transition[TransitionOffset + iTran + 1] = -1;
          }

          pImage = inputImage->imageData - 1 + startCol + (iRow+startRow)*inputImage->widthStep;
          pMask = maskImage->imageData - 1 + iRow*maskImage->widthStep;
        }
      }

      int *SubsumedRegion = NULL;

      double ThisParent, ThisArea, ThisPerimeter, ThisMinX, ThisMaxX, ThisMinY, ThisMaxY, LastPerimeter, ThisExternPerimeter, ThisSumX = 0, ThisSumY = 0, ThisSumXX = 0, ThisSumYY = 0, ThisSumXY = 0;
      int HighRegionNum = 0, ErrorFlag = 0;

      int LastRow, ThisRow;                     // Row number
      int LastStart, ThisStart;                 // Starting column of run
      int LastEnd, ThisEnd;                     // Ending column of run
      int LastColor, ThisColor;                 // Color of run

      int LastIndex, ThisIndex;                 // Which run are we up to
      int LastIndexCount, ThisIndexCount;       // Out of these runs
      int LastRegionNum, ThisRegionNum;         // Which assignment
      int *LastRegion;                          // Row assignment of region number
      int *ThisRegion;                          // Row assignment of region number

      int LastOffset = -(Trans + 2);            // For performance to avoid multiplication
      int ThisOffset = 0;                       // For performance to avoid multiplication
      int ComputeData;

      CvPoint actualedge;
      uchar imagevalue;
      bool CandidatExterior = false;
      CBlob *regionDataThisRegion, *regionDataLastRegion;

      LastRegion=new int[Cols+2];
      ThisRegion=new int[Cols+2];

      for(i = 0; i < Cols + 2; i++)
      {
        LastRegion[i] = -1;
        ThisRegion[i] = -1;
      }

      //create the external blob
      RegionData.push_back( new CBlob() );
      SubsumedRegion = NewSubsume(SubsumedRegion,0);
      RegionData[0]->parent = -1;
      RegionData[0]->area = (double) Transition[0];
      RegionData[0]->perimeter = (double) (2 + 2 * Transition[0]);

      ThisIndexCount = 1;
      ThisRegion[0] = 0;	// Border region

      // beginning of the image
      pImage = inputImage->imageData - 1 + startCol + startRow * inputImage->widthStep;
      //the mask should be the same size as image Roi, so don't take into account the offset
      if(maskImage!=NULL) pMask = maskImage->imageData - 1;

      char *pImageAux, *pMaskAux = NULL;

      // Loop over all rows
      for(ThisRow = 1; ThisRow < Rows + 2; ThisRow++)
      {
        ThisOffset += Trans + 2;
        ThisIndex = 0;
        LastOffset += Trans + 2;;
        LastRow = ThisRow - 1;
        LastIndexCount = ThisIndexCount;
        LastIndex = 0;

        int EndLast = 0;
        int EndThis = 0;

        for(int j = 0; j < Trans + 2; j++)
        {
          int Index = ThisOffset + j;
          int TranVal = Transition[Index];
          if(TranVal > 0) ThisIndexCount = j + 1;

          if(ThisRegion[j] == -1)  { EndLast = 1; }
          if(TranVal < 0) { EndThis = 1; }

          if(EndLast > 0 && EndThis > 0) { break; }

          LastRegion[j] = ThisRegion[j];
          ThisRegion[j] = -1;
        }

        int MaxIndexCount = LastIndexCount;
        if(ThisIndexCount > MaxIndexCount) MaxIndexCount = ThisIndexCount;

        // Main loop over runs within Last and This rows
        while (LastIndex < LastIndexCount && ThisIndex < ThisIndexCount)
        {
          ComputeData = 0;

          if(LastIndex == 0) LastStart = 0;
          else LastStart = Transition[LastOffset + LastIndex - 1];
          LastEnd = Transition[LastOffset + LastIndex] - 1;
          LastColor = LastIndex - 2 * (LastIndex / 2);
          LastRegionNum = LastRegion[LastIndex];

          regionDataLastRegion = RegionData[LastRegionNum];


          if(ThisIndex == 0) ThisStart = 0;
          else ThisStart = Transition[ThisOffset + ThisIndex - 1];
          ThisEnd = Transition[ThisOffset + ThisIndex] - 1;
          ThisColor = ThisIndex - 2 * (ThisIndex / 2);
          ThisRegionNum = ThisRegion[ThisIndex];

          if( ThisRegionNum >= 0 )
            regionDataThisRegion = RegionData[ThisRegionNum];
          else
            regionDataThisRegion = NULL;

          // blobs externs
          CandidatExterior = false;
          if(
    #if !IMATGE_CICLICA_VERTICAL
            ThisRow == 1 || ThisRow == Rows ||
    #endif
    #if !IMATGE_CICLICA_HORITZONTAL
            ThisStart <= 1 || ThisEnd >= Cols ||
    #endif
            GetExternPerimeter( ThisStart, ThisEnd, ThisRow, inputImage->width, inputImage->height, imatgePerimetreExtern )
            )
          {
            CandidatExterior = true;
          }

          int TestA = (LastEnd < ThisStart - 1);                // initially false
          int TestB = (ThisEnd < LastStart);                    // initially false
          int TestC = (LastStart < ThisStart);                  // initially false
          int TestD = (ThisEnd < LastEnd);
          int TestE = (ThisEnd == LastEnd);

          int TestMatch = (ThisColor == LastColor);             // initially true
          int TestKnown = (ThisRegion[ThisIndex] >= 0);         // initially false

          int Case = 0;
          if(TestA) Case = 1;
          else if(TestB) Case = 8;
          else if(TestC)
          {
            if(TestD) Case = 3;
            else if(!TestE) Case = 2;
            else Case = 4;
          }
          else
          {
            if(TestE) Case = 5;
            else if(TestD) Case = 7;
            else Case = 6;
          }

          // Initialize common variables
          ThisArea = (float) 0.0;

          if(findmoments)
          {
            ThisSumX = ThisSumY = (float) 0.0;
            ThisSumXX = ThisSumYY = ThisSumXY = (float) 0.0;
          }
          ThisMinX = ThisMinY = (float) 1000000.0;
          ThisMaxX = ThisMaxY = (float) -1.0;

          LastPerimeter = ThisPerimeter = (float) 0.0;
          ThisParent = (float) -1;
          ThisExternPerimeter = 0.0;

          // Determine necessary action and take it
          switch (Case)
          {
          case 1:
            ThisRegion[ThisIndex] = ThisRegionNum;
            LastRegion[LastIndex] = LastRegionNum;
            LastIndex++;

            actualedge.x = ThisEnd;
            actualedge.y = ThisRow - 1;
            cvSeqPush(regionDataLastRegion->edges,&actualedge);

            actualedge.x = ThisStart - 1;
            actualedge.y = ThisRow - 1;
            cvSeqPush(regionDataThisRegion->edges,&actualedge);

            break;
          case 2:
            if(TestMatch)
            {
              ThisRegionNum = LastRegionNum;
              regionDataThisRegion = regionDataLastRegion;

              ThisArea = ThisEnd - ThisStart + 1;
              LastPerimeter = LastEnd - ThisStart + 1;
              ThisPerimeter = 2 + 2 * ThisArea - LastPerimeter +
                PERIMETRE_DIAGONAL*2;

              if( CandidatExterior )
              {
                ThisExternPerimeter = GetExternPerimeter( ThisStart, ThisEnd, ThisRow,
                  inputImage->width, inputImage->height,
                  imatgePerimetreExtern );
                ThisExternPerimeter += PERIMETRE_DIAGONAL*2;
              }
              ComputeData = 1;
            }

            if(ThisRegionNum!=-1)
            {
              if(ThisStart - 1 != ThisEnd)
              {
                actualedge.x = ThisStart - 1;
                actualedge.y = ThisRow - 1;
                cvSeqPush(regionDataThisRegion->edges,&actualedge);
              }
              actualedge.x = ThisEnd;
              actualedge.y = ThisRow - 1;
              cvSeqPush(regionDataThisRegion->edges,&actualedge);
            }

            if(LastRegionNum!=-1 && LastRegionNum != ThisRegionNum )
            {
              if(ThisStart - 1 != ThisEnd)
              {
                actualedge.x = ThisStart - 1;
                actualedge.y = ThisRow - 1;
                cvSeqPush(regionDataLastRegion->edges,&actualedge);
              }
            }

            ThisRegion[ThisIndex] = ThisRegionNum;
            LastRegion[LastIndex] = LastRegionNum;
            LastIndex++;
            break;
          case 3:
            if(TestMatch)
            {
              ThisRegionNum = LastRegionNum;
              regionDataThisRegion = regionDataLastRegion;

              ThisArea = ThisEnd - ThisStart + 1;
              LastPerimeter = ThisArea;
              ThisPerimeter = 2 + ThisArea + PERIMETRE_DIAGONAL*2;
              if( CandidatExterior )
              {
                ThisExternPerimeter = GetExternPerimeter( ThisStart, ThisEnd, ThisRow,
                  inputImage->width, inputImage->height,
                  imatgePerimetreExtern );

                ThisExternPerimeter += PERIMETRE_DIAGONAL * 2;
              }
            }
            else
            {
              ThisParent = LastRegionNum;
              ThisRegionNum = ++HighRegionNum;
              ThisArea = ThisEnd - ThisStart + 1;
              ThisPerimeter = 2 + 2 * ThisArea;
              RegionData.push_back( new CBlob() );
              regionDataThisRegion = RegionData.back();

              SubsumedRegion = NewSubsume(SubsumedRegion,HighRegionNum);
              if( CandidatExterior )
                ThisExternPerimeter = GetExternPerimeter( ThisStart, ThisEnd, ThisRow,
                inputImage->width, inputImage->height,
                imatgePerimetreExtern );
            }

            if(ThisRegionNum!=-1)
            {
              actualedge.x = ThisStart - 1;
              actualedge.y = ThisRow - 1;
              cvSeqPush(regionDataThisRegion->edges,&actualedge);

              actualedge.x = ThisEnd;
              actualedge.y = ThisRow - 1;
              cvSeqPush(regionDataThisRegion->edges,&actualedge);
            }

            if(!TestMatch && LastRegionNum!=-1 && LastRegionNum != ThisRegionNum )
            {
              actualedge.x = ThisStart - 1;
              actualedge.y = ThisRow - 1;
              cvSeqPush(regionDataLastRegion->edges,&actualedge);

              actualedge.x = ThisEnd;
              actualedge.y = ThisRow - 1;
              cvSeqPush(regionDataLastRegion->edges,&actualedge);
            }

            ThisRegion[ThisIndex] = ThisRegionNum;
            LastRegion[LastIndex] = LastRegionNum;
            ComputeData = 1;
            ThisIndex++;
            break;
          case 4:
            if(TestMatch)
            {
              ThisRegionNum = LastRegionNum;
              regionDataThisRegion = regionDataLastRegion;
              ThisArea = ThisEnd - ThisStart + 1;
              LastPerimeter = ThisArea;
              ThisPerimeter = 2 + ThisArea + PERIMETRE_DIAGONAL;
              if( CandidatExterior )
              {
                ThisExternPerimeter = GetExternPerimeter( ThisStart, ThisEnd, ThisRow,
                  inputImage->width, inputImage->height,
                  imatgePerimetreExtern );

                ThisExternPerimeter += PERIMETRE_DIAGONAL;
              }
            }
            else
            {
              ThisParent = LastRegionNum;
              ThisRegionNum = ++HighRegionNum;
              ThisArea = ThisEnd - ThisStart + 1;
              ThisPerimeter = 2 + 2 * ThisArea;
              RegionData.push_back( new CBlob() );
              regionDataThisRegion = RegionData.back();
              SubsumedRegion = NewSubsume(SubsumedRegion,HighRegionNum);
              if( CandidatExterior )
                ThisExternPerimeter = GetExternPerimeter( ThisStart, ThisEnd, ThisRow,
                inputImage->width, inputImage->height,
                imatgePerimetreExtern );
            }

            if(ThisRegionNum!=-1)
            {
              actualedge.x = ThisStart - 1;
              actualedge.y = ThisRow - 1;
              cvSeqPush(regionDataThisRegion->edges,&actualedge);
              actualedge.x = ThisEnd;
              actualedge.y = ThisRow - 1;
              cvSeqPush(regionDataThisRegion->edges,&actualedge);
            }

            if(!TestMatch && LastRegionNum!=-1 && LastRegionNum != ThisRegionNum )
            {
              actualedge.x = ThisStart - 1;
              actualedge.y = ThisRow - 1;
              cvSeqPush(regionDataLastRegion->edges,&actualedge);
              actualedge.x = ThisEnd;
              actualedge.y = ThisRow - 1;
              cvSeqPush(regionDataLastRegion->edges,&actualedge);
            }

            ThisRegion[ThisIndex] = ThisRegionNum;
            LastRegion[LastIndex] = LastRegionNum;
            ComputeData = 1;

    #ifdef B_CONNECTIVITAT_8
            if( TestMatch )
            {
              LastIndex++;
              ThisIndex++;
            }
            else
            {
              LastIndex++;
            }
    #else
            LastIndex++;
            ThisIndex++;
    #endif
            break;
          case 5:
            if(!TestMatch && !TestKnown)
            {
              ThisParent = LastRegionNum;
              ThisRegionNum = ++HighRegionNum;
              ThisArea = ThisEnd - ThisStart + 1;
              ThisPerimeter = 2 + 2 * ThisArea;
              RegionData.push_back( new CBlob() );
              regionDataThisRegion = RegionData.back();
              SubsumedRegion = NewSubsume(SubsumedRegion,HighRegionNum);
              if( CandidatExterior )
                ThisExternPerimeter = GetExternPerimeter( ThisStart, ThisEnd, ThisRow,
                inputImage->width, inputImage->height,
                imatgePerimetreExtern );

            }
            else if(TestMatch && !TestKnown)
            {
              ThisRegionNum = LastRegionNum;
              regionDataThisRegion = regionDataLastRegion;
              ThisArea = ThisEnd - ThisStart + 1;
              LastPerimeter = LastEnd - LastStart + 1;
              ThisPerimeter = 2 + 2 * ThisArea - LastPerimeter
                + PERIMETRE_DIAGONAL * (LastStart != ThisStart);
              if( CandidatExterior )
              {
                ThisExternPerimeter = GetExternPerimeter( ThisStart, ThisEnd, ThisRow,
                  inputImage->width, inputImage->height,
                  imatgePerimetreExtern );

                ThisExternPerimeter += PERIMETRE_DIAGONAL * (LastStart != ThisStart);
              }
              ComputeData = 1;
            }
            else if(TestMatch && TestKnown)
            {
              LastPerimeter = LastEnd - LastStart + 1;
              ThisPerimeter = - 2 * LastPerimeter
                + PERIMETRE_DIAGONAL * (LastStart != ThisStart);

              if(ThisRegionNum > LastRegionNum)
              {
                Subsume(RegionData, HighRegionNum, SubsumedRegion, regionDataThisRegion, regionDataLastRegion,
                  findmoments, ThisRegionNum, LastRegionNum );
                for(int iOld = 0; iOld < MaxIndexCount; iOld++)
                {
                  if(ThisRegion[iOld] == ThisRegionNum) ThisRegion[iOld] = LastRegionNum;
                  if(LastRegion[iOld] == ThisRegionNum) LastRegion[iOld] = LastRegionNum;
                }
                ThisRegionNum = LastRegionNum;
              }
              else if(ThisRegionNum < LastRegionNum)
              {
                Subsume(RegionData, HighRegionNum, SubsumedRegion, regionDataLastRegion, regionDataThisRegion,
                  findmoments, LastRegionNum, ThisRegionNum );

                for(int iOld = 0; iOld < MaxIndexCount; iOld++)
                {
                  if(ThisRegion[iOld] == LastRegionNum) ThisRegion[iOld] = ThisRegionNum;
                  if(LastRegion[iOld] == LastRegionNum) LastRegion[iOld] = ThisRegionNum;
                }
                LastRegionNum = ThisRegionNum;
              }
            }

            if(ThisRegionNum!=-1)
            {
              actualedge.x = ThisEnd;
              actualedge.y = ThisRow - 1;
              cvSeqPush(regionDataThisRegion->edges,&actualedge);

              if( ThisStart - 1 != LastEnd )
              {
                actualedge.x = ThisStart - 1;
                actualedge.y = ThisRow - 1;
                cvSeqPush(regionDataThisRegion->edges,&actualedge);
              }
            }

            if(!TestMatch && LastRegionNum!=-1 && LastRegionNum != ThisRegionNum )
            {
              actualedge.x = ThisEnd;
              actualedge.y = ThisRow - 1;
              cvSeqPush(regionDataLastRegion->edges,&actualedge);
            }

            ThisRegion[ThisIndex] = ThisRegionNum;
            LastRegion[LastIndex] = LastRegionNum;

    #ifdef B_CONNECTIVITAT_8
            if( TestMatch )
            {
              LastIndex++;
              ThisIndex++;
            }
            else
            {
              LastIndex++;
            }
    #else
            LastIndex++;
            ThisIndex++;
    #endif
            break;
          case 6:
            if(TestMatch && !TestKnown)
            {
              ThisRegionNum = LastRegionNum;
              regionDataThisRegion = regionDataLastRegion;
              ThisArea = ThisEnd - ThisStart + 1;
              LastPerimeter = LastEnd - LastStart + 1;
              ThisPerimeter = 2 + 2 * ThisArea - LastPerimeter
                + PERIMETRE_DIAGONAL + PERIMETRE_DIAGONAL * (ThisStart!=LastStart);
              if( CandidatExterior )
              {
                ThisExternPerimeter = GetExternPerimeter( ThisStart, ThisEnd, ThisRow,
                  inputImage->width, inputImage->height,
                  imatgePerimetreExtern );

                ThisExternPerimeter += PERIMETRE_DIAGONAL + PERIMETRE_DIAGONAL * (ThisStart!=LastStart);
              }
              ComputeData = 1;
            }
            else if(TestMatch && TestKnown)
            {
              LastPerimeter = LastEnd - LastStart + 1;
              ThisPerimeter = - 2 * LastPerimeter
                + PERIMETRE_DIAGONAL + PERIMETRE_DIAGONAL * (ThisStart!=LastStart);

              if(ThisRegionNum > LastRegionNum)
              {
                Subsume(RegionData, HighRegionNum, SubsumedRegion, regionDataThisRegion, regionDataLastRegion,
                  findmoments, ThisRegionNum, LastRegionNum );
                for(int iOld = 0; iOld < MaxIndexCount; iOld++)
                {
                  if(ThisRegion[iOld] == ThisRegionNum) ThisRegion[iOld] = LastRegionNum;
                  if(LastRegion[iOld] == ThisRegionNum) LastRegion[iOld] = LastRegionNum;
                }
                ThisRegionNum = LastRegionNum;
              }
              else if(ThisRegionNum < LastRegionNum)
              {
                Subsume(RegionData, HighRegionNum, SubsumedRegion, regionDataLastRegion, regionDataThisRegion,
                  findmoments, LastRegionNum, ThisRegionNum );
                for(int iOld = 0; iOld < MaxIndexCount; iOld++)
                {
                  if(ThisRegion[iOld] == LastRegionNum) ThisRegion[iOld] = ThisRegionNum;
                  if(LastRegion[iOld] == LastRegionNum) LastRegion[iOld] = ThisRegionNum;
                }
                LastRegionNum = ThisRegionNum;
              }
            }

            if(ThisRegionNum!=-1)
            {
              actualedge.x = ThisEnd;
              actualedge.y = ThisRow - 1;
              cvSeqPush(regionDataThisRegion->edges,&actualedge);
              if( ThisStart - 1 != LastEnd )
              {
                actualedge.x = ThisStart - 1;
                actualedge.y = ThisRow - 1;
                cvSeqPush(regionDataThisRegion->edges,&actualedge);
              }
            }

            if(!TestMatch && LastRegionNum!=-1 && LastRegionNum != ThisRegionNum )
            {
              if( ThisStart - 1 != LastEnd )
              {
                actualedge.x = ThisStart - 1;
                actualedge.y = ThisRow - 1;
                cvSeqPush(regionDataThisRegion->edges,&actualedge);
              }
            }

            ThisRegion[ThisIndex] = ThisRegionNum;
            LastRegion[LastIndex] = LastRegionNum;
            LastIndex++;
            break;
          case 7:
            if(!TestMatch && !TestKnown)
            {
              ThisParent = LastRegionNum;
              ThisRegionNum = ++HighRegionNum;
              ThisArea = ThisEnd - ThisStart + 1;
              ThisPerimeter = 2 + 2 * ThisArea;
              RegionData.push_back( new CBlob() );
              regionDataThisRegion = RegionData.back();
              SubsumedRegion = NewSubsume(SubsumedRegion,HighRegionNum);
              if( CandidatExterior )
                ThisExternPerimeter = GetExternPerimeter( ThisStart, ThisEnd, ThisRow,
                inputImage->width, inputImage->height,
                imatgePerimetreExtern );
            }
            else if(TestMatch && !TestKnown)
            {
              ThisRegionNum = LastRegionNum;
              regionDataThisRegion = regionDataLastRegion;
              ThisArea = ThisEnd - ThisStart + 1;
              ThisPerimeter = 2 + ThisArea;
              LastPerimeter = ThisEnd - LastStart + 1;
              ThisPerimeter = 2 + 2 * ThisArea - LastPerimeter
                + PERIMETRE_DIAGONAL + PERIMETRE_DIAGONAL * (ThisStart!=LastStart);
              if( CandidatExterior )
              {
                ThisExternPerimeter = GetExternPerimeter( ThisStart, ThisEnd, ThisRow,
                  inputImage->width, inputImage->height,
                  imatgePerimetreExtern );

                ThisExternPerimeter += PERIMETRE_DIAGONAL + PERIMETRE_DIAGONAL * (ThisStart!=LastStart);
              }
              ComputeData = 1;
            }
            else if(TestMatch && TestKnown)
            {
              LastPerimeter = ThisEnd - LastStart + 1;
              ThisPerimeter = - 2 * LastPerimeter
                + PERIMETRE_DIAGONAL + PERIMETRE_DIAGONAL * (ThisStart!=LastStart);

              if(ThisRegionNum > LastRegionNum)
              {
                Subsume(RegionData, HighRegionNum, SubsumedRegion, regionDataThisRegion, regionDataLastRegion,
                  findmoments, ThisRegionNum, LastRegionNum );
                for(int iOld = 0; iOld < MaxIndexCount; iOld++)
                {
                  if(ThisRegion[iOld] == ThisRegionNum) ThisRegion[iOld] = LastRegionNum;
                  if(LastRegion[iOld] == ThisRegionNum) LastRegion[iOld] = LastRegionNum;
                }
                ThisRegionNum = LastRegionNum;
              }
              else if(ThisRegionNum < LastRegionNum)
              {
                Subsume(RegionData, HighRegionNum, SubsumedRegion, regionDataLastRegion, regionDataThisRegion,
                  findmoments, LastRegionNum, ThisRegionNum );
                for(int iOld = 0; iOld < MaxIndexCount; iOld++)
                {
                  if(ThisRegion[iOld] == LastRegionNum) ThisRegion[iOld] = ThisRegionNum;
                  if(LastRegion[iOld] == LastRegionNum) LastRegion[iOld] = ThisRegionNum;
                }
                LastRegionNum = ThisRegionNum;
              }
            }

            if(ThisRegionNum!=-1)
            {
              actualedge.x = ThisEnd;
              actualedge.y = ThisRow - 1;
              cvSeqPush(regionDataThisRegion->edges,&actualedge);
              if( ThisStart - 1 != LastEnd )
              {
                actualedge.x = ThisStart - 1;
                actualedge.y = ThisRow - 1;
                cvSeqPush(regionDataThisRegion->edges,&actualedge);
              }
            }

            if(!TestMatch && LastRegionNum!=-1 && LastRegionNum != ThisRegionNum )
            {
              actualedge.x = ThisEnd;
              actualedge.y = ThisRow - 1;
              cvSeqPush(regionDataLastRegion->edges,&actualedge);

              if( ThisStart - 1 != LastEnd )
              {
                actualedge.x = ThisStart - 1;
                actualedge.y = ThisRow - 1;
                cvSeqPush(regionDataThisRegion->edges,&actualedge);
              }
            }

            ThisRegion[ThisIndex] = ThisRegionNum;
            LastRegion[LastIndex] = LastRegionNum;
            ThisIndex++;
            break;
          case 8:
    #ifdef B_CONNECTIVITAT_8

            if( TestMatch )
            {
              if(ThisRegionNum > LastRegionNum)
              {
                Subsume(RegionData, HighRegionNum, SubsumedRegion, regionDataThisRegion, regionDataLastRegion,
                  findmoments, ThisRegionNum, LastRegionNum );
                for(int iOld = 0; iOld < MaxIndexCount; iOld++)
                {
                  if(ThisRegion[iOld] == ThisRegionNum) ThisRegion[iOld] = LastRegionNum;
                  if(LastRegion[iOld] == ThisRegionNum) LastRegion[iOld] = LastRegionNum;
                }
                ThisRegionNum = LastRegionNum;
              }
              else if(ThisRegionNum < LastRegionNum)
              {
                Subsume(RegionData, HighRegionNum, SubsumedRegion, regionDataLastRegion, regionDataThisRegion,
                  findmoments, LastRegionNum, ThisRegionNum );
                for(int iOld = 0; iOld < MaxIndexCount; iOld++)
                {
                  if(ThisRegion[iOld] == LastRegionNum) ThisRegion[iOld] = ThisRegionNum;
                  if(LastRegion[iOld] == LastRegionNum) LastRegion[iOld] = ThisRegionNum;
                }
                LastRegionNum = ThisRegionNum;
              }

              regionDataThisRegion->perimeter = regionDataThisRegion->perimeter + PERIMETRE_DIAGONAL*2;
            }

    #endif

            if(ThisRegionNum!=-1)
            {
              actualedge.x = ThisStart - 1;
              actualedge.y = ThisRow - 1;
              cvSeqPush(regionDataThisRegion->edges,&actualedge);
            }
    #ifdef B_CONNECTIVITAT_8
            if(!TestMatch && LastRegionNum!=-1 && LastRegionNum != ThisRegionNum )
            {
    #endif
              actualedge.x = ThisStart - 1;
              actualedge.y = ThisRow - 1;
              cvSeqPush(regionDataLastRegion->edges,&actualedge);
    #ifdef B_CONNECTIVITAT_8
            }
    #endif

            ThisRegion[ThisIndex] = ThisRegionNum;
            LastRegion[LastIndex] = LastRegionNum;
            ThisIndex++;
    #ifdef B_CONNECTIVITAT_8
            LastIndex--;
    #endif
            break;

          default:
            ErrorFlag = -1;
          }

          // calculate the blob moments and mean gray level of the current blob (ThisRegionNum)
          if(ComputeData > 0)
          {
            // compute blob moments if necessary
            if(findmoments)
            {
              float ImageRow = (float) (ThisRow - 1);

              for(int k = ThisStart; k <= ThisEnd; k++)
              {
                ThisSumX += (float) (k - 1);
                ThisSumXX += (float) (k - 1) * (k - 1);
              }

              ThisSumXY = ThisSumX * ImageRow;
              ThisSumY = ThisArea * ImageRow;
              ThisSumYY = ThisSumY * ImageRow;

            }

            // compute the mean gray level and its std deviation
            if(ThisRow <= Rows )
            {
              pImageAux = pImage + ThisStart;
              if(maskImage!=NULL) pMaskAux = pMask + ThisStart;
              for(int k = ThisStart; k <= ThisEnd; k++)
              {
                if((k>0) && (k <= Cols))
                {
                  if( maskImage!= NULL)
                  {
                    if( ((unsigned char) *pMaskAux) != PIXEL_EXTERIOR )
                    {
                      imagevalue = (unsigned char) (*pImageAux);
                      regionDataThisRegion->mean+=imagevalue;
                      regionDataThisRegion->stddev+=imagevalue*imagevalue;
                    }
                    else
                    {
                      nombre_pixels_mascara++;
                    }
                  }
                  else
                  {
                    imagevalue = (unsigned char) (*pImageAux);
                    regionDataThisRegion->mean+=imagevalue;
                    regionDataThisRegion->stddev+=imagevalue*imagevalue;
                  }
                }
                pImageAux++;
                if(maskImage!=NULL) pMaskAux++;
              }
            }

            // compute the min and max values of X and Y
            if(ThisStart - 1 < (int) ThisMinX) ThisMinX = (float) (ThisStart - 1);
            if(ThisMinX < (float) 0.0) ThisMinX = (float) 0.0;
            if(ThisEnd > (int) ThisMaxX) ThisMaxX = (float) ThisEnd;

            if(ThisRow - 1 < ThisMinY) ThisMinY = ThisRow - 1;
            if(ThisMinY < (float) 0.0) ThisMinY = (float) 0.0;
            if(ThisRow > ThisMaxY) ThisMaxY = ThisRow;
          }

          // put the current results into RegionData
          if(ThisRegionNum >= 0)
          {
            if(ThisParent >= 0) { regionDataThisRegion->parent = (int) ThisParent; }
            regionDataThisRegion->etiqueta = ThisRegionNum;
            regionDataThisRegion->area += ThisArea;
            regionDataThisRegion->perimeter += ThisPerimeter;
            regionDataThisRegion->externPerimeter += ThisExternPerimeter;

            if(ComputeData > 0)
            {
              if(findmoments)
              {
                regionDataThisRegion->sumx += ThisSumX;
                regionDataThisRegion->sumy += ThisSumY;
                regionDataThisRegion->sumxx += ThisSumXX;
                regionDataThisRegion->sumyy += ThisSumYY;
                regionDataThisRegion->sumxy += ThisSumXY;
              }
              regionDataThisRegion->perimeter -= LastPerimeter;
              regionDataThisRegion->minx=MIN(regionDataThisRegion->minx,ThisMinX);
              regionDataThisRegion->maxx=MAX(regionDataThisRegion->maxx,ThisMaxX);
              regionDataThisRegion->miny=MIN(regionDataThisRegion->miny,ThisMinY);
              regionDataThisRegion->maxy=MAX(regionDataThisRegion->maxy,ThisMaxY);
            }

            // blobs externs
            if( CandidatExterior )
            {
              regionDataThisRegion->exterior = true;
            }

          }
        }	// end Main loop

        if(ErrorFlag != 0) return false;
        pImage = inputImage->imageData - 1 + startCol + (ThisRow+startRow) * inputImage->widthStep;

        if(maskImage!=NULL)
          pMask = maskImage->imageData - 1 + ThisRow * maskImage->widthStep;
      }	// end Loop over all rows

      RegionData[0]->area -= ( Rows + 1 + Cols + 1 )*2 + nombre_pixels_mascara;
      RegionData[0]->perimeter -= 8.0;

      // Condense the list
      blob_vector::iterator itNew, itOld, iti;
      CBlob *blobActual;

      itNew = RegionData.begin();
      itOld = RegionData.begin();
      int iNew = 0;
      for(int iOld = 0; iOld <= HighRegionNum; iOld++, itOld++)
      {
        if(SubsumedRegion[iOld] < 1)	// This number not subsumed
        {
          **itNew = **itOld;

          // Update and parent pointer if necessary
          iti = RegionData.begin();
          for(int i = 0; i <= HighRegionNum; i++)
          {
            if((*iti)->parent == iOld) { (*iti)->parent = iNew; }
            iti++;
          }
          iNew++;
          itNew++;
        }
      }

      HighRegionNum = iNew - 1;                 // Update where the data ends
      RegionData[HighRegionNum]->parent = -1;	// and set end of array flag


      if(findmoments)
      {
        iti = RegionData.begin();

        for(ThisRegionNum = 0; ThisRegionNum <= HighRegionNum; ThisRegionNum++, iti++)
        {
          blobActual = *iti;

          // Get averages
          blobActual->sumx /= blobActual->area;
          blobActual->sumy /= blobActual->area;
          blobActual->sumxx /= blobActual->area;
          blobActual->sumyy /= blobActual->area;
          blobActual->sumxy /= blobActual->area;

          // Create moments
          blobActual->sumxx -= blobActual->sumx * blobActual->sumx;
          blobActual->sumyy -= blobActual->sumy * blobActual->sumy;
          blobActual->sumxy -= blobActual->sumx * blobActual->sumy;
          if(blobActual->sumxy > -1.0E-14 && blobActual->sumxy < 1.0E-14)
          {
            blobActual->sumxy = (float) 0.0;
          }
        }
      }

      //Get the real mean and std deviation
      iti = RegionData.begin();
      for(ThisRegionNum = 0; ThisRegionNum <= HighRegionNum; ThisRegionNum++, iti++)
      {
        blobActual = *iti;
        if(blobActual->area > 1)
        {
          blobActual->stddev =
            sqrt(
            (
            blobActual->stddev * blobActual->area -
            blobActual->mean * blobActual->mean
            )/
            (blobActual->area*(blobActual->area-1))
            );
        }
        else
          blobActual->stddev=0;

        if(blobActual->area > 0)
          blobActual->mean/=blobActual->area;
        else
          blobActual->mean = 0;

      }
      // eliminem els blobs subsumats
      blob_vector::iterator itBlobs = RegionData.begin() + HighRegionNum + 1;
      while( itBlobs != RegionData.end() )
      {
        delete *itBlobs;
        itBlobs++;
      }
      RegionData.erase( RegionData.begin() + HighRegionNum + 1, RegionData.end() );

      free(SubsumedRegion);
      delete Transition;
      delete ThisRegion;
      delete LastRegion;

      if( imatgePerimetreExtern ) cvReleaseImage(&imatgePerimetreExtern);

      return true;
    }

    int *NewSubsume(int *subsumed, int index_subsume)
    {
      if( index_subsume == 0 )
      {
        subsumed = (int*)malloc(sizeof(int));
      }
      else
      {
        subsumed = (int*)realloc(subsumed,(index_subsume+1)*sizeof(int));
      }
      subsumed[index_subsume]=0;
      return subsumed;
    }

    void Subsume(blob_vector &RegionData,
      int HighRegionNum,
      int* SubsumedRegion,
      CBlob* blobHi,
      CBlob* blobLo,
      bool findmoments,
      int HiNum,
      int LoNum)
    {
      int i;

      blobLo->minx=MIN(blobHi->minx,blobLo->minx);
      blobLo->miny=MIN(blobHi->miny,blobLo->miny);
      blobLo->maxx=MAX(blobHi->maxx,blobLo->maxx);
      blobLo->maxy=MAX(blobHi->maxy,blobLo->maxy);
      blobLo->area+=blobHi->area;
      blobLo->perimeter+=blobHi->perimeter;
      blobLo->externPerimeter += blobHi->externPerimeter;
      blobLo->exterior = blobLo->exterior || blobHi->exterior;
      blobLo->mean += blobHi->mean;
      blobLo->stddev += blobHi->stddev;

      if( findmoments )
      {
        blobLo->sumx+=blobHi->sumx;
        blobLo->sumy+=blobHi->sumy;
        blobLo->sumxx+=blobHi->sumxx;
        blobLo->sumyy+=blobHi->sumyy;
        blobLo->sumxy+=blobHi->sumxy;
      }

      blob_vector::iterator it = (RegionData.begin() + HiNum + 1);

      for(i = HiNum + 1; i <= HighRegionNum; i++, it++)
      {
        if((*it)->parent == (float) HiNum) { (*it)->parent = LoNum; }
      }

      SubsumedRegion[HiNum] = 1;
      blobHi->etiqueta=-1;

      blobHi->CopyEdges( *blobLo );
      blobHi->ClearEdges();
    }

    double GetExternPerimeter( int start, int end, int row, int width, int height, IplImage *imatgePerimetreExtern )
    {
      double perimeter = 0.0f;
      char *pPerimetre;

      perimeter += ( start <= 0 ) + ( end >= width - 1 );
      if(row <= 1 ) perimeter+= start - end;
      if(row >= height - 1 ) perimeter+= start - end;

      if( imatgePerimetreExtern != NULL )
      {
        if( row <= 0 || row >= height ) return perimeter;

        if( start < 0 ) start = 1;
        if( end >= width ) end = width - 2;

        pPerimetre = imatgePerimetreExtern->imageData + (row - 1) * imatgePerimetreExtern->widthStep + start;
        for (int x = start - 1; x <= end; x++ )
        {
          perimeter += *pPerimetre;
          pPerimetre++;
        }
      }

      return perimeter;
    }
}
