// https://medium.com/@shishir_dey/upload-data-to-google-sheet-with-an-esp32-and-some-scripting-2d8b0ccbc833

function doGet(e){
  // open the spreadsheet
  var ss = SpreadsheetApp.getActive();
  
  // use the 'id' parameter to differentiate between sheets
  var sheet = ss.getSheetByName(e.parameter["id"]);
  
  // extract headers
  // getRange accepts row, col, number_of_rows and num_of_cols as argument
  // getLastColumn returns the position of the last column that has content
  var headers = sheet.getRange(1, 1, 1, sheet.getLastColumn()).getValues()[0];
  
  // store the position of the last row
  var lastRow = sheet.getLastRow();
  
  var cell = sheet.getRange('a1');
  var col = 0;
  var d = new Date();
  
  for (i in headers){
    
    // loop through the headers and if a parameter name matches the header name insert the value
    if (headers[i] == "Timestamp")
    {
      val = d.toDateString() + ", " + d.toLocaleTimeString();
    }
    else
    {
      val = e.parameter[headers[i]]; 
    }
    
    // append data to the last row
    cell.offset(lastRow, col).setValue(val);
    col++;
  }
  
  return ContentService.createTextOutput('success');
}