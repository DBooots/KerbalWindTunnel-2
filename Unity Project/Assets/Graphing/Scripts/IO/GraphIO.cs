using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Graphing.IO
{
    public static class GraphIO
    {
        public const string defaultSheetName = "Sheet1";

        public static readonly char[] invalidSheetChars = new char[] { '[', ']', '*', '/', '\\', '?', ':' };
        public enum FileFormat
        {
            CSV = 1,
            [Obsolete]
            XLS = 2,
            XLSX = 4,
            PNG = 8,
            JPG = 16,
            Image = PNG | JPG,
#pragma warning disable CS0612 // Type or member is obsolete
            Excel = XLS | XLSX
#pragma warning restore CS0612 // Type or member is obsolete
        }

        public static readonly SpreadsheetOptions defaultOptions = new SpreadsheetOptions(true, true, 1, 0);

        /// <summary>
        /// Gets or sets the spreadsheet writer class. Can be initialized with <see cref="InitializeMiniExcelWriter"/>.
        /// </summary>
        public static ISpreadsheetWriter SpreadsheetWriter { get; set; }

        /// <summary>
        /// Outputs the object's values to file.
        /// </summary>
        /// <param name="graph">The graph object to write.</param>
        /// <param name="directory">The directory in which to place the file.</param>
        /// <param name="filename">The filename for the file.</param>
        /// <param name="format">The format to be written.</param>
        /// <param name="sheetName">An optional sheet name for within the file.</param>
        /// <param name="resolution">An optional value for the resolution of the target image.</param>
        /// <exception cref="System.NotImplementedException">
        /// Image format is not supported.
        /// or
        /// File format is not supported.
        /// </exception>
        public static void WriteToFile(this IGraphable graph, string directory, string filename, FileFormat format = FileFormat.CSV, string sheetName = "", (int width, int height)? resolution = null)
        {
            string path = InitializeAndGetPath(directory, filename, sheetName, format);
            switch (format)
            {
                case FileFormat.CSV:
                    graph.WriteToFileCSV(path);
                    break;
#pragma warning disable CS0612 // Type or member is obsolete
                case FileFormat.XLS:
#pragma warning restore CS0612 // Type or member is obsolete
                case FileFormat.XLSX:
                    if (string.IsNullOrEmpty(sheetName))
                        sheetName = graph.DisplayName;
                    if (string.IsNullOrEmpty(sheetName) && !(graph is GraphableCollection))
                        sheetName = defaultSheetName;
                    sheetName = StripInvalidSheetChars(sheetName);
                    graph.WriteToFileXLS(path, sheetName);
                    break;
                case FileFormat.PNG:
                case FileFormat.JPG:
                    Texture2D texture = new Texture2D(resolution?.width ?? 1024, resolution?.height ?? 1024, TextureFormat.ARGB32, false);
                    graph.Draw(ref texture, graph.XMin, graph.XMax, graph.YMin, graph.YMax);
                    byte[] byteStream;
                    switch (format)
                    {
                        case FileFormat.PNG:
                            byteStream = texture.EncodeToPNG();
                            break;
                        case FileFormat.JPG:
                            byteStream = texture.EncodeToJPG();
                            break;
                        default:
                            throw new NotImplementedException("Image format is not supported.");
                    }
                    System.IO.File.WriteAllBytes(path, byteStream);
                    UnityEngine.Object.Destroy(texture);
                    break;
                default:
                    throw new NotImplementedException("File format is not supported.");
            }
        }

        /// <summary>
        /// Outputs a collection's values to file.
        /// </summary>
        /// <param name="collection">The collection to write.</param>
        /// <param name="directory">The directory in which to place the file.</param>
        /// <param name="filename">The filename for the file.</param>
        /// <param name="visibleOnly">If set to <c>true</c> only writes the visible children of the collection.</param>
        /// <param name="format">The format to be written.</param>
        /// <param name="sheetName">An optional sheet name for within the file.</param>
        /// <param name="resolution">An optional value for the resolution of the target image.</param>
        /// <exception cref="System.NotImplementedException">File format is not supported.</exception>
        public static void WriteToFile(this GraphableCollection collection, string directory, string filename, bool visibleOnly, FileFormat format = FileFormat.CSV, string sheetName = "", (int width, int height)? resolution = null)
        {
            if ((format & FileFormat.Image) > 0)
            {
                WriteToFile(collection, directory, filename, format, sheetName, resolution);
                return;
            }

            string path = InitializeAndGetPath(directory, filename, sheetName, format);
            switch (format)
            {
                case FileFormat.CSV:
                    collection.WriteToFileCSV(path, visibleOnly);
                    break;
#pragma warning disable CS0612 // Type or member is obsolete
                case FileFormat.XLS:
#pragma warning restore CS0612 // Type or member is obsolete
                case FileFormat.XLSX:
                    if (string.IsNullOrEmpty(sheetName))
                        sheetName = collection.DisplayName;
                    if (string.IsNullOrEmpty(sheetName))
                        sheetName = defaultSheetName;
                    sheetName = StripInvalidSheetChars(sheetName);
                    collection.WriteToFileXLS(path, sheetName, visibleOnly);
                    break;
                default:
                    throw new NotImplementedException("File format is not supported.");
            }
        }

        private static string InitializeAndGetPath(string directory, string filename, string sheetName, FileFormat format)
        {
            if (!System.IO.Directory.Exists(directory))
                System.IO.Directory.CreateDirectory(directory);

            string path = ValidateFilePath(directory, filename, format);

            if (format == FileFormat.CSV && !string.IsNullOrEmpty(sheetName))
            {
                sheetName = sheetName.Replace("/", "-").Replace("\\", "-");
                sheetName = StripInvalidFileChars(sheetName);
                path.Insert(path.Length - 4, sheetName);
            }

            try
            {
                if (System.IO.File.Exists(path))
                    System.IO.File.Delete(path);
            }
            catch (Exception ex) { Debug.Log($"Unable to delete file:{ex.Message}"); }

            return path;
        }

        /// <summary>
        /// Outputs the grapher's RenderTexture values to file.
        /// </summary>
        /// <param name="directory">The directory in which to place the file.</param>
        /// <param name="filename">The filename for the file.</param>
        /// <param name="format">The format to be written.</param>
        /// <exception cref="System.ArgumentException">Format must be an image format.</exception>
        /// <exception cref="System.NotImplementedException">Image format is not supported.</exception>
        public static void WriteRendering(this Grapher grapher, string directory, string filename, FileFormat format)
        {
            if (format != FileFormat.PNG && format != FileFormat.JPG)
                throw new ArgumentException("Format must be an image format.", nameof(format));

            if (!System.IO.Directory.Exists(directory))
                System.IO.Directory.CreateDirectory(directory);

            string path = ValidateFilePath(directory, filename, format);

            try
            {
                if (System.IO.File.Exists(path))
                    System.IO.File.Delete(path);
            }
            catch (Exception ex) { Debug.Log($"Unable to delete file:{ex.Message}"); }

            RenderTexture active = RenderTexture.active;
            RenderTexture renderTexture = grapher.GetComponentInChildren<Camera>(true).targetTexture;
            RenderTexture.active = renderTexture;
            Texture2D texture = new Texture2D(renderTexture.width, renderTexture.height, TextureFormat.ARGB32, false);
            texture.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0);
            texture.Apply();
            RenderTexture.active = active;

            byte[] byteStream;
            switch (format)
            {
                case FileFormat.PNG:
                    byteStream = texture.EncodeToPNG();
                    break;
                case FileFormat.JPG:
                    byteStream = texture.EncodeToJPG();
                    break;
                default:
                    throw new NotImplementedException("Image format is not supported.");
            }
            System.IO.File.WriteAllBytes(path, byteStream);
            UnityEngine.Object.Destroy(texture);
        }

        public static string ValidateFilePath(string directory, string filename, string format)
        {
            if (string.IsNullOrEmpty(directory))
                throw new ArgumentNullException(nameof(directory));
            if (string.IsNullOrEmpty(filename))
                throw new ArgumentNullException(nameof(filename));
            if (!PathCharsAreValid(directory))
                throw new System.IO.DirectoryNotFoundException(directory);
            int extensionIndex = filename.LastIndexOf('.');
            string extension;
            if (extensionIndex == -1)
                extension = format;
            else
            {
                extension = filename.Substring(extensionIndex);
                if (!string.Equals(extension, format, StringComparison.OrdinalIgnoreCase))
                    extension = format;
                else
                    filename = filename.Substring(0, extensionIndex);
            }
            filename = StripInvalidFileChars(filename);
            if (string.IsNullOrEmpty(filename))
                throw new ArgumentException("The resulting filename is empty", nameof(filename));
            filename = string.Concat(filename, extension);
            return string.Join($"{System.IO.Path.DirectorySeparatorChar}", directory, filename);
        }
        public static string ValidateFilePath(string directory, string filename, FileFormat format)
        {
            string formatExtension;
            switch (format)
            {
                case FileFormat.CSV:
                    formatExtension = ".csv";
                    break;
#pragma warning disable CS0612 // Type or member is obsolete
                case FileFormat.XLS:
#pragma warning restore CS0612 // Type or member is obsolete
                    formatExtension = ".xls";
                    break;
                case FileFormat.XLSX:
                    formatExtension = ".xlsx";
                    break;
                case FileFormat.PNG:
                    formatExtension = ".png";
                    break;
                case FileFormat.JPG:
                    formatExtension = ".jpg";
                    break;
                default:
                    throw new NotImplementedException("File format extension is missing");
            }
            return ValidateFilePath(directory, filename, formatExtension);
        }

        public static bool PathCharsAreValid(string directory)
        {
            char[] invalidChars = System.IO.Path.GetInvalidPathChars();
            foreach (char c in directory.ToCharArray())
                if (invalidChars.Contains(c))
                    return false;
            return true;
        }

        public static string StripInvalidFileChars(string filename)
        {
            if (filename == null)
                throw new ArgumentNullException(nameof(filename));
            char[] invalidChars = System.IO.Path.GetInvalidFileNameChars();
            char[] chars = filename.ToCharArray();
            for (int i = chars.Length - 1; i >= 0; i--)
                if (invalidChars.Contains(chars[i]))
                    filename.Remove(i, 1);
            return filename;
        }

        public static string StripInvalidSheetChars(string sheetName)
        {
            if (sheetName == null)
                throw new ArgumentNullException(nameof(sheetName));
            sheetName = sheetName.Replace("/", "-").Replace("\\", "-");
            char[] chars = sheetName.ToCharArray();
            for (int i = chars.Length - 1; i >= 0; i--)
                if (invalidSheetChars.Contains(chars[i]))
                    sheetName.Remove(i, 1);
            return sheetName;
        }

        public static string GetUniqueName(IEnumerable<string> usedNames, string name)
        {
            if (!usedNames.Contains(name))
                return name;
            int i = 1;
            while (usedNames.Contains(string.Concat(name, i.ToString())))
                i++;
            return string.Concat(name, i.ToString());
        }
        public static string GetUniqueColumnName(System.Data.DataTable dataTable, string name)
        {
            return GetUniqueName(dataTable.Columns.ColumnNames(), name);
        }
        public static IEnumerable<string> ColumnNames(this System.Data.DataColumnCollection collection)
        {
            foreach (System.Data.DataColumn column in collection)
                yield return column.ColumnName;
        }
        public static string GetValidSheetName(IGraphable graph)
        {
            string sheetName = graph.DisplayName.Replace("/", "-").Replace("\\", "-");
            if (sheetName.Length >= 30)
                sheetName = graph.Name.Replace("/", "-").Replace("\\", "-");
            if (string.IsNullOrWhiteSpace(sheetName))
                sheetName = graph.GetType().Name;
            sheetName = StripInvalidSheetChars(sheetName);
            return sheetName;
        }
    }
}
