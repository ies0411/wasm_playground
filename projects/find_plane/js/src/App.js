import logo from './logo.svg';
import { Button, Upload } from 'antd';
import { UploadOutlined } from '@ant-design/icons';
import createModule from './wasm/findplane';

import './App.css';

function App() {
  let pcdArray = [];
  let doubletype = 8;
  const solve = (e) => {
    e.preventDefault();
    createModule().then((Module) => {
      console.log('check');
      let res_ptr = Module._malloc(4 * 2);
      // console.log(res_ptr[0]);
      let getImageResolution = () => null;
      console.log('check');

      getImageResolution = Module.cwrap('intrinsicGetImageResolution', 'number', ['string', 'number']);
      console.log('check');
      const filename = 'test';
      getImageResolution(filename, res_ptr);
      console.log('check');

      // Module._getImageResolution(fileName, res_ptr);
      // console.log(res_ptr[0]);
      // var output_array = new Int32Array(Module.HEAP32.buffer, output_ptr, len)
      let [width, height] = new Int32Array(Module.HEAP32.buffer, res_ptr, 2);
      console.log('wid : ' + width);
      console.log('hei :' + height);

      pcdArray.forEach((pcdData, index) => {
        //write file
        let fileName = `pcd${index}`;
        // new Uint8Array(data)
        console.log('js pcd :' + fileName);
        Module.FS.writeFile(fileName, new Uint8Array(pcdData));

        //read pcd and size
        let readPCD = () => null;
        let pcd_size_ptr = Module._malloc(doubletype * 2);
        readPCD = Module.cwrap('readPCD', 'number', ['string', 'number']);
        readPCD(fileName, pcd_size_ptr);
        let weight_height = new Float64Array(Module.HEAPF64.buffer, pcd_size_ptr, 2);
        console.log('width : ' + weight_height[0]);
        console.log('height : ' + weight_height[1]);

        //get decoded pcd
        let pcd_size = weight_height[0] * weight_height[1];
        let pcd_ptr = Module._malloc(pcd_size * 4 * doubletype);
        Module._decodePCD(pcd_ptr);
        let decoded_pcd_data = new Float64Array(Module.HEAPF64.buffer, pcd_ptr, pcd_size * 4);
        console.log(decoded_pcd_data);

        //get plane coeff
        let coeff_plane_ptr = Module._malloc(doubletype * 4);
        Module._findPlane(coeff_plane_ptr);
        let plane_coeff = new Float64Array(Module.HEAPF64.buffer, coeff_plane_ptr, 4);
        console.log(plane_coeff);

        //Free malloc memory
        Module._free(pcd_ptr);
        Module._free(coeff_plane_ptr);
        Module._free(pcd_size_ptr);
      });
    });
  };
  const pcd_props = {
    accept: '.pcd',
    multiple: true,
    showUploadList: false,
    beforeUpload: (file) => {
      const reader = new FileReader();
      // readAsArrayBuffer
      // reader.readAsText(file);
      reader.readAsArrayBuffer(file);

      reader.onload = () => {
        pcdArray.push(reader.result);
      };
    },
  };
  return (
    <div className="App">
      <div id="file">
        <Upload {...pcd_props}>
          <Button icon={<UploadOutlined />}>PCD Upload</Button>
        </Upload>
      </div>
      <div id="solve">
        <Button onClick={solve}>solve</Button>
      </div>

      <header className="App-header">
        <img src={logo} className="App-logo" alt="logo" />
        <p>
          Edit <code>src/App.js</code> and save to reload.
        </p>
        <a className="App-link" href="https://reactjs.org" target="_blank" rel="noopener noreferrer">
          Learn React
        </a>
      </header>
    </div>
  );
}

export default App;
