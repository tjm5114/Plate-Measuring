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
    using System.Drawing;
    using AForge;
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

                float xDist = P1.X - corner.X;
                float yDist = P1.Y - corner.Y;
                float distance = P1.DistanceTo(corner);
                System.Console.WriteLine("The Straight Line Distance Between P1 and Point {0} is {1}", corner, distance);
                System.Console.WriteLine("The X  Distance Between P1 and Point {0} is {1}", corner, xDist);
                System.Console.WriteLine("The Y Line Distance Between P1 and Point {0} is {1}", corner, yDist);
            }

            PointsMarker marker = new PointsMarker(corners, System.Drawing.Color.Green,4);

            // Apply the corner-marking filter
            Bitmap markers = marker.Apply(myimage);

            markers.Save("c:\\Users\\Main\\Documents\\myNewimage.png", System.Drawing.Imaging.ImageFormat.Png);

            
            
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
}
