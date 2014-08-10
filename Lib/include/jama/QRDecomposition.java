package Jama;

import Jama.util.Maths;
import java.io.Serializable;

public class QRDecomposition
  implements Serializable
{
  private double[][] QR;
  private int m;
  private int n;
  private double[] Rdiag;

  public QRDecomposition(Matrix paramMatrix)
  {
    this.QR = paramMatrix.getArrayCopy();
    this.m = paramMatrix.getRowDimension();
    this.n = paramMatrix.getColumnDimension();
    this.Rdiag = new double[this.n];

    for (int i = 0; i < this.n; ++i)
    {
      double d1 = 0D;
      for (int j = i; j < this.m; ++j) {
        d1 = Maths.hypot(d1, this.QR[j][i]);
      }

      if (d1 != 0D)
      {
        if (this.QR[i][i] < 0D)
          d1 = -d1;

        for (j = i; j < this.m; ++j) {
          int tmp124 = i;
          double[] tmp124 = this.QR[j]; tmp124[tmp124] = (tmp124[tmp124] / d1);
        }
        int tmp142 = i;
        double[] tmp142 = this.QR[i]; tmp142[tmp142] = (tmp142[tmp142] + 1D);

        for (j = i + 1; j < this.n; ++j) {
          double d2 = 0D;
          for (int k = i; k < this.m; ++k)
            d2 = d2 + this.QR[k][i] * this.QR[k][j];

          d2 = -d2 / this.QR[i][i];
          for (k = i; k < this.m; ++k) {
            int tmp242 = j;
            double[] tmp242 = this.QR[k]; tmp242[tmp242] = (tmp242[tmp242] + d2 * this.QR[k][i]);
          }
        }
      }
      this.Rdiag[i] = (-d1);
    }
  }

  public boolean isFullRank()
  {
    for (int i = 0; i < this.n; ++i)
      if (this.Rdiag[i] == 0D)
        return false;

    return true;
  }

  public Matrix getH()
  {
    Matrix localMatrix = new Matrix(this.m, this.n);
    double[][] arrayOfDouble = localMatrix.getArray();
    for (int i = 0; i < this.m; ++i)
      for (int j = 0; j < this.n; ++j)
        if (i >= j)
          arrayOfDouble[i][j] = this.QR[i][j];
        else
          arrayOfDouble[i][j] = 0D;



    return localMatrix;
  }

  public Matrix getR()
  {
    Matrix localMatrix = new Matrix(this.n, this.n);
    double[][] arrayOfDouble = localMatrix.getArray();
    for (int i = 0; i < this.n; ++i)
      for (int j = 0; j < this.n; ++j)
        if (i < j)
          arrayOfDouble[i][j] = this.QR[i][j];
        else if (i == j)
          arrayOfDouble[i][j] = this.Rdiag[i];
        else
          arrayOfDouble[i][j] = 0D;



    return localMatrix;
  }

  public Matrix getQ()
  {
    Matrix localMatrix = new Matrix(this.m, this.n);
    double[][] arrayOfDouble = localMatrix.getArray();
    for (int i = this.n - 1; i >= 0; --i) {
      for (int j = 0; j < this.m; ++j)
        arrayOfDouble[j][i] = 0D;

      arrayOfDouble[i][i] = 1D;
      for (j = i; j < this.n; ++j)
        if (this.QR[i][i] != 0D) {
          double d = 0D;
          for (int k = i; k < this.m; ++k)
            d = d + this.QR[k][i] * arrayOfDouble[k][j];

          d = -d / this.QR[i][i];
          for (k = i; k < this.m; ++tmp163) {
            int tmp163 = j;
            double[] tmp163 = arrayOfDouble[k]; tmp163[tmp163] = (tmp163[tmp163] + d * this.QR[tmp163][i]);
          }
        }
    }

    return localMatrix;
  }

  public Matrix solve(Matrix paramMatrix)
  {
    if (paramMatrix.getRowDimension() != this.m)
      throw new IllegalArgumentException("Matrix row dimensions must agree.");

    if (!(isFullRank())) {
      throw new RuntimeException("Matrix is rank deficient.");
    }

    int i = paramMatrix.getColumnDimension();
    double[][] arrayOfDouble = paramMatrix.getArrayCopy();

    for (int j = 0; j < this.n; ++j)
      for (int k = 0; k < i; ++k) {
        double d = 0D;
        for (int i3 = j; i3 < this.m; ++i3)
          d = d + this.QR[i3][j] * arrayOfDouble[i3][k];

        d = -d / this.QR[j][j];
        for (i3 = j; i3 < this.m; ++i3) {
          int tmp149 = k;
          double[] tmp149 = arrayOfDouble[i3]; tmp149[tmp149] = (tmp149[tmp149] + d * this.QR[i3][j]);
        }
      }


    for (j = this.n - 1; j >= 0; --j) {
      for (int l = 0; l < i; ++l) {
        int tmp212 = l;
        double[] tmp212 = arrayOfDouble[j]; tmp212[tmp212] = (tmp212[tmp212] / this.Rdiag[j]);
      }
      for (int i1 = 0; i1 < j; ++i1)
        for (int i2 = 0; i2 < i; ++i2) {
          int tmp254 = i2;
          double[] tmp254 = arrayOfDouble[i1]; tmp254[tmp254] = (tmp254[tmp254] - arrayOfDouble[j][i2] * this.QR[i1][j]);
        }
    }

    return new Matrix(arrayOfDouble, this.n, i).getMatrix(0, this.n - 1, 0, i - 1);
  }
}
