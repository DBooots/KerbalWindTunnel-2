using System;
using System.IO;
using System.Linq;
using System.Runtime.Serialization;
using System.Threading.Tasks;
using Graphing.IO;
using UnityEngine;

namespace KerbalWindTunnel
{
    public class MiniExcelWrapper : ISpreadsheetWriter, IDisposable
    {
        private readonly AppDomain excelDomain;
        private IDomainWriter worker;
        public MiniExcelWrapper()
        {
            excelDomain = AppDomain.CreateDomain("Excel Domain");

            string path = typeof(MiniExcelWrapper).Assembly.Location;
            char pathChar = Path.DirectorySeparatorChar;
            
            string[] dirs = Directory.GetFiles(path.Substring(0, path.LastIndexOf(pathChar)), "*.dll*", SearchOption.AllDirectories);
            static bool ContainsNameString(string filename) => filename.Contains("MiniExcelDomainWorker");
            // Use the autoloaded one only if we absolutely have to. Prefer the ".dll.noautoload" variant.
            // People may not have deleted the earlier .dll extension one and that one won't be updated.
            string dir = dirs.Where(s => !s.EndsWith(".dll", StringComparison.InvariantCultureIgnoreCase)).FirstOrDefault(ContainsNameString) ??
                dirs.FirstOrDefault(ContainsNameString);
            worker = (IDomainWriter)excelDomain.CreateInstanceFromAndUnwrap(dir, "MiniExcelDomain.MiniExcelWorker");

            path = string.Join($"{pathChar}", path.Substring(0, path.LastIndexOf(pathChar)), "References");
            Exception loadException = worker.SetupDomain(path);
            if (loadException != null)
                throw new AggregateException(loadException);

            Debug.LogFormat("[KWT] MiniExcel AppDomain created, these are the assemblies it has:\n{0}", worker.ListAssemblies());
        }

        public void Write(string path, string sheet, object data, SpreadsheetOptions? options = null)
        {
            if (!(data is ISerializable))
                throw new SerializationException($"{nameof(data)} must implement {nameof(ISerializable)} in order to cross to the other AppDomain.");
            if (worker == null)
                throw new ObjectDisposedException(nameof(worker));
#if DEBUG
            Debug.Log("[KWT] Writing data.");
#endif
            try
            {
                Exception exception = worker.Write(path, sheet, data, options);
                if (exception != null)
                    throw new AggregateException(exception);
            }
            catch (Exception ex)
            {
                Debug.LogException(ex);
            }
#if DEBUG
            Debug.Log("[KWT] Done writing data.");
#endif
        }

        public async Task WriteToCSV(string path, System.Data.DataTable table, bool printHeader, string sheetName = "")
        {
            if (worker == null)
                throw new ObjectDisposedException(nameof(worker));
#if DEBUG
            Debug.Log("[KWT] Writing data.");
#endif
            try
            {
                Task result = Task.Run(() => worker.WriteToCSV(path, table, printHeader, sheetName));
                await result;
                if (result.Exception != null)
                    throw new AggregateException(result.Exception);
            }
            catch (Exception ex)
            {
                Debug.LogException(ex);
            }
#if DEBUG
            Debug.Log("[KWT] Done writing data.");
#endif
        }

        public void Dispose()
        {
            worker = null;
            AppDomain.Unload(excelDomain);
        }
    }

    public interface IDomainWriter
    {
        string ListAssemblies();
        Exception SetupDomain(string path);
#if DEBUG
        bool CheckAssembly(string assemblyName);
#endif
        Exception Write(string path, string sheet, object data, SpreadsheetOptions? options = null);
        void WriteToCSV(string path, System.Data.DataTable table, bool printHeader, string sheetName = "");
    }
}
