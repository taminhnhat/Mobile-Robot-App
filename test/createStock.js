const axios = require('axios');
const productList = require('./productList')

function putToLight() {
    axios.post('http://192.168.1.42:80/api/v1/putToLight/', {
        userId: "Minh_Nhat",
        productId: productList[Math.floor(Math.random() * 30)],
        binId: `M-${Math.floor(Math.random() * 24 + 1)}-${Math.floor(Math.random() * 18 + 1)}`,
        productQuantity: Math.floor(Math.random() * 100 + 1)
    }, {
        headers: {
            api_key: 'mgw_cEfRlzOgO2EwRe9ha7Ho'
        }
    })
        .then(response => {
            console.log(response.data)
        })
        .catch(error => {
            console.log(error.message);
        });
}

putToLight()