//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.DepthBasics
{
    using System;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using System.Windows.Forms;
    using Microsoft.Kinect;
    using System.Collections.Generic;
    using System.Drawing;
    using System.Drawing.Imaging;
    using System;
    using Accord.Math;
    using AForge;
    using AForge.Imaging;
    using AForge.Imaging.Filters;
    using Accord.Imaging;
    using Accord.Imaging.Filters;


    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor sensor;

        /// <summary>
        /// Bitmap that will hold color information
        /// </summary>
        private WriteableBitmap colorBitmap;

        /// <summary>
        /// Intermediate storage for the depth data received from the camera
        /// </summary>
        private short[] depthPixels;

        /// <summary>
        /// Intermediate storage for the depth data converted to color
        /// </summary>
        private byte[] colorPixels;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            InitializeComponent();
        }

        /// <summary>
        /// Execute startup tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            // Look through all sensors and start the first connected one.
            // This requires that a Kinect is connected at the time of app startup.
            // To make your app robust against plug/unplug, 
            // it is recommended to use KinectSensorChooser provided in Microsoft.Kinect.Toolkit
            foreach (var potentialSensor in KinectSensor.KinectSensors)
            {
                if (potentialSensor.Status == KinectStatus.Connected)
                {
                    this.sensor = potentialSensor;
                    break;
                }
            }

            if (null != this.sensor)
            {
                // Turn on the depth stream to receive depth frames
                this.sensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);

                // Allocate space to put the depth pixels we'll receive
                this.depthPixels = new short[this.sensor.DepthStream.FramePixelDataLength];

                // Allocate space to put the color pixels we'll create
                this.colorPixels = new byte[this.sensor.DepthStream.FramePixelDataLength * sizeof(int)];

                // This is the bitmap we'll display on-screen
                this.colorBitmap = new WriteableBitmap(this.sensor.DepthStream.FrameWidth, this.sensor.DepthStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);

                // Set the image we display to point to the bitmap where we'll put the image data
                this.Image.Source = this.colorBitmap;

                // Add an event handler to be called whenever there is new depth frame data
                this.sensor.DepthFrameReady += this.SensorDepthFrameReady;

                // Start the sensor!
                try
                {
                    this.sensor.Start();
                }
                catch (IOException)
                {
                    this.sensor = null;
                }
            }

            if (null == this.sensor)
            {
                this.statusBarText.Text = Properties.Resources.NoKinectReady;
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (null != this.sensor)
            {
                this.sensor.Stop();
            }
        }

        /// <summary>
        /// Event handler for Kinect sensor's DepthFrameReady event
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void SensorDepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                if (depthFrame != null)
                {
                    //for (int j = 0; j < 100; j++)
                    //{
                        // Copy the pixel data from the image to a temporary array
                        depthFrame.CopyPixelDataTo(this.depthPixels);

                        // Convert the depth to RGB
                        int colorPixelIndex = 0;
                        for (int i = 0; i < this.depthPixels.Length; ++i)
                        {
                            // discard the portion of the depth that contains only the player index
                            short depth = (short)(this.depthPixels[i] >> DepthImageFrame.PlayerIndexBitmaskWidth);

                            // to convert to a byte we're looking at only the lower 8 bits
                            // by discarding the most significant rather than least significant data
                            // we're preserving detail, although the intensity will "wrap"
                            // add 1 so that too far/unknown is mapped to black
                            byte intensity = (byte)((depth + 1) & byte.MaxValue);

                            // Write out blue byte
                            this.colorPixels[colorPixelIndex++] = intensity;

                            // Write out green byte
                            this.colorPixels[colorPixelIndex++] = intensity;

                            // Write out red byte                        
                            this.colorPixels[colorPixelIndex++] = intensity;

                            // We're outputting BGR, the last byte in the 32 bits is unused so skip it
                            // If we were outputting BGRA, we would write alpha here.
                            ++colorPixelIndex;
                        }

                        // Write the pixel data into our bitmap
                        this.colorBitmap.WritePixels(
                            new Int32Rect(0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight),
                            this.colorPixels,
                            this.colorBitmap.PixelWidth * sizeof(int),
                            0);
                    //}
                }
            }
        }

        /// <summary>
        /// Handles the user clicking on the screenshot button
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void ButtonScreenshotClick(object sender, RoutedEventArgs e)
        {
            Bitmap myimage = new Bitmap("c:\\Octave\\scripts\\mybasicimage.png");
            PictureBox pictureBox1 = new System.Windows.Forms.PictureBox();
            double sigma = 1.2;
            float k = 0.4f;
            float threshold = 200000f;

            // Create a new Harris Corners Detector using the given parameters
            HarrisCornersDetector fast = new HarrisCornersDetector();
            //{
            //    Suppress = true, // suppress non-maximum points
            //    Threshold = 40   // less leads to more corners
            //};


            System.Collections.Generic.List<AForge.IntPoint> corners = fast.ProcessImage(myimage);
            //Process points

            //To-Do create edge detection matrix
            
            //To-Do start with initial point

            //Starting with your intial point in the corner's matrix, look around intial points for points in the edge detection matrix with similar y components

            //Take the distance between your intial point and your second point in the corner's matrix

            //From this second point disregard the previous path and check now the x components in the edge detection matrix for a path to our next point in the corner's matrix

            //Take the distance between the second point and our new third point

            //Rinse and Repeat till end condition is met

            //Once the edge is found go to the po
            //create 
            IntPoint P1 = new IntPoint(0, 0);
            foreach( AForge.IntPoint corner in corners)
            {
                float distance = P1.DistanceTo(corner);
                System.Console.WriteLine("The Distance between P1 some point is ", distance);
            }

            PointsMarker marker = new PointsMarker(corners, System.Drawing.Color.Green,4);

            // Apply the corner-marking filter
            Bitmap markers = marker.Apply(myimage);

            markers.Save("c:\\Users\\Main\\Documents\\myNewimage.png", System.Drawing.Imaging.ImageFormat.Png);

            // Show on the screen
           // ImageBox.Show(markers);
            //foreach (AForge.IntPoint corner in corners)
           // {
            ////    
           // }

            // Create a new AForge's Corner Marker Filter
           // CornersMarker corners = new CornersMarker(harris, System.Drawing.Color.Red);

            // Apply the filter and display it on a picturebox
           // Bitmap myNewimage = corners.Apply(myimage);
            
            if (null == this.sensor)
            {
                this.statusBarText.Text = Properties.Resources.ConnectDeviceFirst;
                return;
            }

            // create a png bitmap encoder which knows how to save a .png file
            BitmapEncoder encoder = new PngBitmapEncoder();

            // create frame from the writable bitmap and add to encoder
            //encoder.Frames.Add(BitmapFrame.Create(this.colorBitmap));
            encoder.Frames.Add(BitmapFrame.Create(this.colorBitmap));
           // myNewimage.Save("c:\\Users\\Main\\Documents\\myNewimage.png", System.Drawing.Imaging.ImageFormat.Png);
            



            string time = System.DateTime.Now.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);

            string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);

            string path = Path.Combine(myPhotos, "PartMeasurement-" + time + ".png");

            // write the new file to disk
            try
            {
                using (FileStream fs = new FileStream(path, FileMode.Create))
                {
                    encoder.Save(fs);
                }

                this.statusBarText.Text = string.Format("{0} {1}", Properties.Resources.ScreenshotWriteSuccess, path);
            }
            catch (IOException)
            {
                this.statusBarText.Text = string.Format("{0} {1}", Properties.Resources.ScreenshotWriteFailed, path);
            }
        }

        /// <summary>
        /// Handles the checking or unchecking of the near mode combo box
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void CheckBoxNearModeChanged(object sender, RoutedEventArgs e)
        {
            if (this.sensor != null)
            {
                // will not function on non-Kinect for Windows devices
                try
                {
                    if (this.checkBoxNearMode.IsChecked.GetValueOrDefault())
                    {
                        this.sensor.DepthStream.Range = DepthRange.Near;
                    }
                    else
                    {
                        this.sensor.DepthStream.Range = DepthRange.Default;
                    }
                }
                catch (InvalidOperationException)
                {
                }
            }
        }
    }

    public enum HarrisCornerMeasure
    {
        /// <summary>
        ///   Original Harris' measure. Requires the setting of
        ///   a parameter k (default is 0.04), which may be a
        ///   bit arbitrary and introduce more parameters to tune.
        /// </summary>
        /// 
        Harris,

        /// <summary>
        ///   Noble's measure. Does not require a parameter
        ///   and may be more stable.
        /// </summary>
        /// 
        Noble,
    }

    public class HarrisCornersDetector : ICornersDetector
    {
        //Harris parameters
        private HarrisCornerMeasure measure = HarrisCornerMeasure.Harris;
        private float k = 0.04f;
        private float threshold = 200000f;

        //Non-maximum suppression parameters
        private int r = 3;

        //Gaussian smooth parameters
        private double sigma = 1.2;
        private float[] kernel;
        private int size = 7;

        //Gets or sets the measure to use when detecting corners
        public HarrisCornerMeasure Measure
        {
            get { return measure; }
            set { measure = value; }
        }

        public float K
        {
            get { return k; }
            set { k = value; }
        }

        public float Threshold
        {
            get { return threshold; }
            set { threshold = value; }
        }

        /// <summary>
        ///   Gaussian smoothing sigma. Default value is 1.2.
        /// </summary>
        /// 
        public double Sigma
        {
            get { return sigma; }
            set
            {
                if (sigma != value)
                {
                    sigma = value;
                    createGaussian();
                }
            }
        }

        /// <summary>
        ///   Non-maximum suppression window radius. Default value is 3.
        /// </summary>
        /// 
        public int Suppression
        {
            get { return r; }
            set { r = value; }
        }

        #region Constructors

        /// <summary>
        ///   Initializes a new instance of the <see cref="HarrisCornersDetector"/> class.
        /// </summary>
        public HarrisCornersDetector()
        {
            initialize(HarrisCornerMeasure.Harris, k, threshold, sigma, r, size);
        }

        /// <summary>
        ///   Initializes a new instance of the <see cref="HarrisCornersDetector"/> class.
        /// </summary>
        public HarrisCornersDetector(float k)
        {
            initialize(HarrisCornerMeasure.Harris, k, threshold, sigma, r, size);
        }

        /// <summary>
        ///   Initializes a new instance of the <see cref="HarrisCornersDetector"/> class.
        /// </summary>
        public HarrisCornersDetector(float k, float threshold)
        {
            initialize(HarrisCornerMeasure.Harris, k, threshold, sigma, r, size);
        }

        /// <summary>
        ///   Initializes a new instance of the <see cref="HarrisCornersDetector"/> class.
        /// </summary>
        public HarrisCornersDetector(float k, float threshold, double sigma)
        {
            initialize(HarrisCornerMeasure.Harris, k, threshold, sigma, r, size);
        }

        /// <summary>
        ///   Initializes a new instance of the <see cref="HarrisCornersDetector"/> class.
        /// </summary>
        public HarrisCornersDetector(float k, float threshold, double sigma, int suppression)
        {
            initialize(HarrisCornerMeasure.Harris, k, threshold, sigma, suppression, size);
        }

        /// <summary>
        ///   Initializes a new instance of the <see cref="HarrisCornersDetector"/> class.
        /// </summary>
        public HarrisCornersDetector(HarrisCornerMeasure measure, float threshold, double sigma, int suppression)
        {
            initialize(measure, k, threshold, sigma, suppression, size);
        }

        /// <summary>
        ///   Initializes a new instance of the <see cref="HarrisCornersDetector"/> class.
        /// </summary>
        public HarrisCornersDetector(HarrisCornerMeasure measure, float threshold, double sigma)
        {
            initialize(measure, k, threshold, sigma, r, size);
        }

        /// <summary>
        ///   Initializes a new instance of the <see cref="HarrisCornersDetector"/> class.
        /// </summary>
        public HarrisCornersDetector(HarrisCornerMeasure measure, float threshold)
        {
            initialize(measure, k, threshold, sigma, r, size);
        }

        /// <summary>
        ///   Initializes a new instance of the <see cref="HarrisCornersDetector"/> class.
        /// </summary>
        public HarrisCornersDetector(HarrisCornerMeasure measure)
        {
            initialize(measure, k, threshold, sigma, r, size);
        }

        private void initialize(HarrisCornerMeasure measure, float k,
            float threshold, double sigma, int suppression, int size)
        {
            this.measure = measure;
            this.threshold = threshold;
            this.k = k;
            this.r = suppression;
            this.sigma = sigma;
            this.size = size;

            createGaussian();
        }

        private void createGaussian()
        {
            double[] aforgeKernel = new AForge.Math.Gaussian(sigma).Kernel(size);
            this.kernel = Array.ConvertAll<double, float>(aforgeKernel, Convert.ToSingle);
        }
        #endregion

         /// <summary>
        ///   Process image looking for corners.
        /// </summary>
        /// 
        /// <param name="image">Source image data to process.</param>
        /// 
        /// <returns>Returns list of found corners (X-Y coordinates).</returns>
        /// 
        /// <exception cref="UnsupportedImageFormatException">
        ///   The source image has incorrect pixel format.
        /// </exception>
        /// 
        public unsafe List<IntPoint> ProcessImage(UnmanagedImage image)
        {
            
            // check image format
            if (
                (image.PixelFormat != System.Drawing.Imaging.PixelFormat.Format8bppIndexed) &&
                (image.PixelFormat != System.Drawing.Imaging.PixelFormat.Format24bppRgb) &&
                (image.PixelFormat != System.Drawing.Imaging.PixelFormat.Format32bppRgb) &&
                (image.PixelFormat != System.Drawing.Imaging.PixelFormat.Format32bppArgb)
                )
            {
                throw new UnsupportedImageFormatException("Unsupported pixel format of the source image.");
            }

            // make sure we have grayscale image
            UnmanagedImage grayImage = null;

            if (image.PixelFormat == System.Drawing.Imaging.PixelFormat.Format8bppIndexed)
            {
                grayImage = image;
            }
            else
            {
                // create temporary grayscale image
                grayImage = Grayscale.CommonAlgorithms.BT709.Apply(image);
            }


            // get source image size
            int width = grayImage.Width;
            int height = grayImage.Height;
            int stride = grayImage.Stride;
            int offset = stride - width;


            // 1. Calculate partial differences
            float[,] diffx = new float[height, width];
            float[,] diffy = new float[height, width];
            float[,] diffxy = new float[height, width];


            fixed (float* pdx = diffx, pdy = diffy, pdxy = diffxy)
            {
                // Begin skipping first line
                byte* src = (byte*)grayImage.ImageData.ToPointer() + stride;
                float* dx = pdx + width;
                float* dy = pdy + width;
                float* dxy = pdxy + width;

                // for each line
                for (int y = 1; y < height - 1; y++)
                {
                    // skip first column
                    dx++; dy++; dxy++; src++;

                    // for each inner pixel in line (skipping first and last)
                    for (int x = 1; x < width - 1; x++, src++, dx++, dy++, dxy++)
                    {
                        // Retrieve the pixel neighborhood
                        byte a11 = src[+stride + 1], a12 = src[+1], a13 = src[-stride + 1];
                        byte a21 = src[+stride + 0], /*  a22    */  a23 = src[-stride + 0];
                        byte a31 = src[+stride - 1], a32 = src[-1], a33 = src[-stride - 1];

                        // Convolution with horizontal differentiation kernel mask
                        float h = ((a11 + a12 + a13) - (a31 + a32 + a33)) * 0.166666667f;

                        // Convolution with vertical differentiation kernel mask
                        float v = ((a11 + a21 + a31) - (a13 + a23 + a33)) * 0.166666667f;

                        // Store squared differences directly
                        *dx = h * h;
                        *dy = v * v;
                        *dxy = h * v;
                    }

                    // Skip last column
                    dx++; dy++; dxy++; 
                    src += offset + 1;
                }

                // Free some resources which wont be needed anymore
                if (image.PixelFormat != System.Drawing.Imaging.PixelFormat.Format8bppIndexed)
                    grayImage.Dispose();
            }


            // 2. Smooth the diff images
            if (sigma > 0.0)
            {
                float[,] temp = new float[height, width];

                // Convolve with Gaussian kernel
                convolve(diffx, temp, kernel);
                convolve(diffy, temp, kernel);
                convolve(diffxy, temp, kernel);
            }


            // 3. Compute Harris Corner Response Map
            float[,] map = new float[height, width];

            fixed (float* pdx = diffx, pdy = diffy, pdxy = diffxy, pmap = map)
            {
                float* dx = pdx;
                float* dy = pdy;
                float* dxy = pdxy;
                float* H = pmap;
                float M, A, B, C;

                for (int y = 0; y < height; y++)
                {
                    for (int x = 0; x < width; x++, dx++, dy++, dxy++, H++)
                    {
                        A = *dx;
                        B = *dy;
                        C = *dxy;

                        if (measure == HarrisCornerMeasure.Harris)
                        {
                            // Original Harris corner measure
                            M = (A * B - C * C) - (k * ((A + B) * (A + B)));
                        }
                        else
                        {
                            // Harris-Noble corner measure
                            M = (A * B - C * C) / (A + B + Constants.SingleEpsilon);
                        }

                        if (M > threshold)
                        {
                            *H = M; // insert value in the map
                        }
                    }
                }
            }


            // 4. Suppress non-maximum points
            List<IntPoint> cornersList = new List<IntPoint>();

            // for each row
            for (int y = r, maxY = height - r; y < maxY; y++)
            {
                // for each pixel
                for (int x = r, maxX = width - r; x < maxX; x++)
                {
                    float currentValue = map[y, x];

                    // for each windows' row
                    for (int i = -r; (currentValue != 0) && (i <= r); i++)
                    {
                        // for each windows' pixel
                        for (int j = -r; j <= r; j++)
                        {
                            if (map[y + i, x + j] > currentValue)
                            {
                                currentValue = 0;
                                break;
                            }
                        }
                    }

                    // check if this point is really interesting
                    if (currentValue != 0)
                    {
                        cornersList.Add(new IntPoint(x, y));
                    }
                }
            }


            return cornersList;
        }

        /// <summary>
        ///   Convolution with decomposed 1D kernel.
        /// </summary>
        /// 
        private static void convolve(float[,] image, float[,] temp, float[] kernel)
        {
            int width = image.GetLength(1);
            int height = image.GetLength(0);
            int radius = kernel.Length / 2;

            unsafe
            {
                fixed (float* ptrImage = image, ptrTemp = temp)
                {
                    float* src = ptrImage + radius;
                    float* tmp = ptrTemp + radius;

                    for (int y = 0; y < height; y++)
                    {
                        for (int x = radius; x < width - radius; x++, src++, tmp++)
                        {
                            float v = 0;
                            for (int k = 0; k < kernel.Length; k++)
                                v += src[k - radius] * kernel[k];
                            *tmp = v;
                        }
                        src += 2 * radius;
                        tmp += 2 * radius;
                    }


                    for (int x = 0; x < width; x++)
                    {
                        for (int y = radius; y < height - radius; y++)
                        {
                            src = ptrImage + y * width + x;
                            tmp = ptrTemp + y * width + x;

                            float v = 0;
                            for (int k = 0; k < kernel.Length; k++)
                                v += tmp[width * (k - radius)] * kernel[k];
                            *src = v;
                        }
                    }
                }
            }
        }


        /// <summary>
        ///   Process image looking for corners.
        /// </summary>
        /// 
        /// <param name="imageData">Source image data to process.</param>
        /// 
        /// <returns>Returns list of found corners (X-Y coordinates).</returns>
        /// 
        /// <exception cref="UnsupportedImageFormatException">
        ///   The source image has incorrect pixel format.
        /// </exception>
        /// 
        public List<IntPoint> ProcessImage(BitmapData imageData)
        {
            return ProcessImage(new UnmanagedImage(imageData));
        }

        /// <summary>
        ///   Process image looking for corners.
        /// </summary>
        /// 
        /// <param name="image">Source image data to process.</param>
        /// 
        /// <returns>Returns list of found corners (X-Y coordinates).</returns>
        /// 
        /// <exception cref="UnsupportedImageFormatException">
        ///   The source image has incorrect pixel format.
        /// </exception>
        /// 
        public List<IntPoint> ProcessImage(Bitmap image)
        {
            // check image format
            if (
                (image.PixelFormat != System.Drawing.Imaging.PixelFormat.Format8bppIndexed) &&
                (image.PixelFormat != System.Drawing.Imaging.PixelFormat.Format24bppRgb) &&
                (image.PixelFormat != System.Drawing.Imaging.PixelFormat.Format32bppRgb) &&
                (image.PixelFormat != System.Drawing.Imaging.PixelFormat.Format32bppArgb)
                )
            {
                throw new UnsupportedImageFormatException("Unsupported pixel format of the source");
            }

            // lock source image
            BitmapData imageData = image.LockBits(
                new Rectangle(0, 0, image.Width, image.Height),
                ImageLockMode.ReadOnly, image.PixelFormat);

            List<IntPoint> corners;

            try
            {
                // process the image
                corners = ProcessImage(new UnmanagedImage(imageData));
            }
            finally
            {
                // unlock image
                image.UnlockBits(imageData);
            }

            return corners;
        }
    
        

    }
}